/*******************************************************************************
 *  Copyright (c) 2016
 *  Capable Humanitarian Robotics and Intelligent Systems Lab (CHRISLab)
 *  Christopher Newport University
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *    1. Redistributions of source code must retain the above copyright notice,
 *       this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 *    3. Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 *       THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *       "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *       LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *       FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *       COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *       INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *       BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *       LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *       CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *       LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
 *       WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *       POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#include <flex_nav_planners/get_path.h>

#include <geometry_msgs/Twist.h>
#include <vector>

using costmap_2d::NO_INFORMATION;

namespace flex_nav {
GetPath::GetPath(tf2_ros::Buffer &tf)
    : tf_(tf), gp_server_(NULL), costmap_(NULL),
      loader_("nav_core", "nav_core::BaseGlobalPlanner"),
      name_(ros::this_node::getName()) {
  ros::NodeHandle private_nh("~");
  ros::NodeHandle nh;

  gp_server_ = new GetPathActionServer(
      nh, name_, boost::bind(&GetPath::execute, this, _1), false);
  cc_server_ = new ClearCostmapActionServer(
      nh, name_ + "/clear_costmap",
      boost::bind(&GetPath::clear_costmap, this, _1), false);

  std::string planner;
  private_nh.param("planner", planner, std::string("navfn/NavfnROS"));
  private_nh.param("costmap/robot_base_frame", robot_base_frame_,
                   std::string("base_link"));
  private_nh.param("costmap/global_frame", global_frame_, std::string("/map"));
  private_nh.param("planner_frequency", planner_frequency_, 1.0);

  costmap_ = new costmap_2d::Costmap2DROS("global_costmap", tf);
  costmap_->pause();

  try {
    planner_ = loader_.createInstance(planner);
    planner_->initialize(loader_.getName(planner), costmap_);
    ROS_INFO("[%s] Created instance of %s planner", name_.c_str(),
             planner.c_str());
  } catch (const pluginlib::PluginlibException &ex) {
    ROS_ERROR("[%s] Failed to create the %s planner: %s", name_.c_str(),
              planner.c_str(), ex.what());
    exit(1);
  }

  costmap_->start();
  gp_server_->start();
  cc_server_->start();
}

GetPath::~GetPath() {
  gp_server_->shutdown();
  cc_server_->shutdown();
}

void GetPath::execute(const flex_nav_common::GetPathGoalConstPtr &goal) {
  geometry_msgs::PoseStamped start;
  costmap_->getRobotPose(start);

  ROS_INFO("[%s] Current location (%f, %f)", name_.c_str(),
           start.pose.position.x, start.pose.position.y);
  ROS_INFO("[%s] Generating a path to (%f, %f), qz=%f", name_.c_str(),
           goal->pose.pose.position.x, goal->pose.pose.position.y,
           goal->pose.pose.orientation.z);

  flex_nav_common::GetPathFeedback feedback;
  feedback.location = start.pose;
  feedback.goal = goal->pose.pose;
  gp_server_->publishFeedback(feedback);

  std::vector<geometry_msgs::PoseStamped> plan;
  flex_nav_common::GetPathResult result;
  result.plan.header.stamp = ros::Time::now();
  if (planner_->makePlan(start, goal->pose, plan)) {
    result.plan.poses = plan;
    if (!plan.empty()) {
      result.code = flex_nav_common::GetPathResult::SUCCESS;
      gp_server_->setSucceeded(result, "Found a path");
    } else {
      result.code = flex_nav_common::GetPathResult::EMPTY;
      gp_server_->setAborted(result, "Empty path");
    }
  } else {
    result.code = flex_nav_common::GetPathResult::FAILURE;
    gp_server_->setAborted(result, "Failed to make plan");
  }
}

void GetPath::clear_costmap(
    const flex_nav_common::ClearCostmapGoalConstPtr &goal) {
  costmap_->resetLayers();

  flex_nav_common::ClearCostmapResult result;
  result.code = flex_nav_common::ClearCostmapResult::SUCCESS;
  cc_server_->setSucceeded(result, "Success");
}
}
