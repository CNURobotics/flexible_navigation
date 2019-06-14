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

#include <base_local_planner/goal_functions.h>
#include <flex_nav_planners/follow_common.h>
#include <flex_nav_planners/follow_path.h>

#include <geometry_msgs/Twist.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace flex_nav {
FollowPath::FollowPath(tf2_ros::Buffer &tf)
    : tf_(tf), fp_server_(NULL), costmap_(NULL),
      loader_("nav_core", "nav_core::BaseGlobalPlanner"), running_(false),
      name_(ros::this_node::getName()) {
  ros::NodeHandle private_nh("~");
  ros::NodeHandle nh;

  fp_server_ = new FollowPathActionServer(
      nh, name_, boost::bind(&FollowPath::execute, this, _1), false);
  cc_server_ = new ClearCostmapActionServer(
      nh, name_ + "/clear_costmap",
      boost::bind(&FollowPath::clear_costmap, this, _1), false);

  std::string planner;
  private_nh.param("planner", planner, std::string("navfn/NavfnROS"));
  private_nh.param("costmap_name", costmap_name_,
                   std::string("middle_costmap"));
  private_nh.param(costmap_name_ + "/robot_base_frame", robot_base_frame_,
                   std::string("base_link"));
  private_nh.param(costmap_name_ + "/reference_frame", global_frame_,
                   std::string("/odom"));
  private_nh.param("planner_frequency", planner_frequency_, 1.0);
  private_nh.param("distance_threshold", distance_threshold_, 5.0);

  costmap_ = new costmap_2d::Costmap2DROS(costmap_name_, tf);
  costmap_->pause();

  try {
    ROS_INFO("[%s] Create instance of %s planner", name_.c_str(),
             planner.c_str());
    planner_ = loader_.createInstance(planner);
    planner_->initialize(loader_.getName(planner), costmap_);
  } catch (const pluginlib::PluginlibException &ex) {
    ROS_ERROR("[%s] Failed to create the %s planner: %s", name_.c_str(),
              planner.c_str(), ex.what());
    exit(1);
  }

  costmap_->start();
  fp_server_->start();
  cc_server_->start();
  ros::spin();
}

FollowPath::~FollowPath() {
  fp_server_->shutdown();
  cc_server_->shutdown();
}

void FollowPath::execute(const flex_nav_common::FollowPathGoalConstPtr &goal) {
  ros::Rate r(planner_frequency_);
  flex_nav_common::FollowPathResult result;

  ROS_INFO(" [%s] Received goal  (%d)", name_.c_str(),
           fp_server_->isNewGoalAvailable());

  while (running_) {
    ROS_WARN_THROTTLE(0.25, "[%s] Waiting for lock", name_.c_str());
  }

  while (fp_server_->isNewGoalAvailable()) {
    ROS_WARN_THROTTLE(0.25, "[%s] Waiting for prior goal to clear ",
                     name_.c_str());
  }

  if (goal->path.poses.empty()) {
    ROS_ERROR("[%s] The path is empty!", name_.c_str());
    result.code = flex_nav_common::FollowPathResult::FAILURE;
    fp_server_->setAborted(result, "Path is empty!");
    return;
  }

  if (goal->path.poses[0].header.frame_id == "") {
    ROS_ERROR("[%s] The frame_id is empty!", name_.c_str());
    result.code = flex_nav_common::FollowPathResult::FAILURE;
    fp_server_->setAborted(result, "frame_id is empty!");
    return;
  }

  ROS_INFO(
      " [%s] Ready to process latest goal  New Goal(%d) Preempt Requested(%d)",
      name_.c_str(), fp_server_->isNewGoalAvailable(),
      fp_server_->isPreemptRequested());

  running_ = true;
  ros::NodeHandle n;
  while (running_ && n.ok() && !fp_server_->isNewGoalAvailable() &&
         !fp_server_->isPreemptRequested()) {
    geometry_msgs::PoseStamped start_pose_map;
    geometry_msgs::PoseStamped start_pose_path;
    geometry_msgs::PoseStamped goal_pose_map;
    geometry_msgs::PoseStamped goal_pose_path;

    // Get the current robot pose from costmap
    costmap_->getRobotPose(start_pose_map);

    start_pose_path.header.stamp = start_pose_map.header.stamp;
    // Transform the pose into the same frame as path to follow
    if (!transformRobot(tf_, start_pose_map, start_pose_path, goal->path.poses[0].header.frame_id))
    {
      ROS_ERROR("[%s] No valid starting point found along path", name_.c_str());
      result.code = flex_nav_common::FollowPathResult::FAILURE;
      fp_server_->setAborted(result, "Failed to transform starting pose!");
      running_ = false;
      return;

    }

    // Update result with latest robot pose information
    result.pose   = start_pose_path.pose;

    // Do some work to find the goal point along the desired path
    costmap_2d::Costmap2D *costmap = costmap_->getCostmap();
    double r2 =
        std::min(costmap->getSizeInCellsX() * costmap->getResolution() / 2.0,
                 costmap->getSizeInCellsY() * costmap->getResolution() / 2.0) -
        costmap->getResolution() * 2;
    r2 = r2 * r2;

    if (!getTargetPointFromPath(r2, start_pose_path, goal->path.poses,
                                goal_pose_path)) {
      ROS_ERROR("[%s] No valid point found - cannot get a valid goal along path",
                name_.c_str());
      result.code = flex_nav_common::FollowPathResult::FAILURE;
      fp_server_->setAborted(result, "Failed to get a valid goal along path");
      running_ = false;
      return;
    }

    // Convert the goal point from path frame to map frame
    goal_pose_map.header.stamp = ros::Time();
    goal_pose_map.header.frame_id = global_frame_;
    result.pose = goal_pose_path.pose;

    if (!transformRobot(tf_, goal_pose_path, goal_pose_map, global_frame_))
    {
      ROS_ERROR("[%s] No valid transform for the goal point found along path", name_.c_str());
      result.code = flex_nav_common::FollowPathResult::FAILURE;
      fp_server_->setAborted(result, "Failed to transform goal pose!");
      running_ = false;
      return;
    }

    double threshold = distance_threshold_ * costmap->getResolution();
    if (distanceSquared(start_pose_path, goal_pose_path) <= threshold * threshold) {
      ROS_INFO("[%s] Reached goal - Success!", name_.c_str());
      result.code = flex_nav_common::FollowPathResult::SUCCESS;
      fp_server_->setSucceeded(result, "Success!");
      running_ = false;
      return;
    }

    // Update the status of robot while following the path
    flex_nav_common::FollowPathFeedback feedback;
    feedback.pose = goal_pose_path.pose;
    fp_server_->publishFeedback(feedback);

    std::vector<geometry_msgs::PoseStamped> plan;
    if (planner_->makePlan(goal_pose_map, goal_pose_map, plan)) {
      if (plan.empty()) {
        ROS_WARN("[%s] Empty path - abort goal!", name_.c_str());
        result.code = flex_nav_common::FollowPathResult::FAILURE; // empty plan
        fp_server_->setAborted(result, "Empty path");
        running_ = false;
        return;
      }
    } else {
      ROS_WARN("[%s] Failed to make plan - abort goal!", name_.c_str());
      result.code =
          flex_nav_common::FollowPathResult::FAILURE; // failed to make plan
      fp_server_->setAborted(result, "Failed to make plan");
      running_ = false;
      return;
    }

    r.sleep();
  }

  if (fp_server_->isPreemptRequested()) {
    ROS_WARN("[%s] Preempt requested - preempting goal ...", name_.c_str());
    fp_server_->setPreempted(result, "Preempting goal");
  } else if (fp_server_->isNewGoalAvailable()) {
    ROS_WARN("[%s] New goal available  - preempting goal ...", name_.c_str());
    fp_server_->setPreempted(result, "Preempting goal");
  } else {
    ROS_ERROR("[%s] Aborting for unknown reason!", name_.c_str());
    fp_server_->setAborted(result, " Aborting for unknown reason!");
  }
  running_ = false;
}

void FollowPath::clear_costmap(
    const flex_nav_common::ClearCostmapGoalConstPtr &goal) {
  costmap_->resetLayers();

  flex_nav_common::ClearCostmapResult result;
  result.code = flex_nav_common::ClearCostmapResult::SUCCESS;
  cc_server_->setSucceeded(result, "Success");
}
}
