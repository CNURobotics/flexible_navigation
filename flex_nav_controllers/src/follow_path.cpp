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

#include <flex_nav_controllers/follow_path.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


namespace flex_nav {
FollowPath::FollowPath(tf2_ros::Buffer &tf)
    : tf_(tf), fp_server_(NULL), costmap_(NULL),
      loader_("nav_core", "nav_core::BaseLocalPlanner"), running_(false),
      name_(ros::this_node::getName()) {
  ros::NodeHandle private_nh("~");
  ros::NodeHandle nh;

  fp_server_ = new FollowPathActionServer(
      nh, name_, boost::bind(&FollowPath::execute, this, _1), false);
  cc_server_ = new ClearCostmapActionServer(
      nh, name_ + "/clear_costmap",
      boost::bind(&FollowPath::clear_costmap, this, _1), false);

  std::string planner;
  private_nh.param("planner", planner,
                   std::string("base_local_planner/TrajectoryPlannerROS"));
  private_nh.param("controller_frequency", controller_frequency_, 5.0);
  private_nh.param("robot_frame", robot_frame_, std::string("base_footprint"));

  vel_pub_ = nh.advertise<geometry_msgs::TwistStamped>("cmd_vel", 1);

  costmap_ = new costmap_2d::Costmap2DROS("local_costmap", tf_);
  costmap_->pause();

  try {
    planner_ = loader_.createInstance(planner);
    planner_->initialize(loader_.getName(planner), &tf_, costmap_);
    ROS_INFO("[%s] Created instance of %s planner", name_.c_str(),
             planner.c_str());
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
  ros::Rate r(controller_frequency_);
  geometry_msgs::PoseStamped location;
  geometry_msgs::PoseStamped pose;

  while (running_) {
    ROS_WARN_THROTTLE(0.25, "[%s] Waiting for lock", name_.c_str());
    r.sleep();
  }

  if (!planner_->setPlan(goal->path.poses)) {
    costmap_->getRobotPose(pose);

    flex_nav_common::FollowPathResult abort;
    abort.code =
        flex_nav_common::FollowPathResult::FAILURE; // Could not set plan
    abort.pose = location.pose;

    fp_server_->setAborted(abort, "Could not set plan");
  }

  ROS_INFO("[%s] Executing new path", name_.c_str());

  running_ = true;
  ros::NodeHandle n;
  while (running_ && n.ok() && !fp_server_->isNewGoalAvailable() &&
         !fp_server_->isPreemptRequested()) {
    costmap_->getRobotPose(pose);

    if (planner_->isGoalReached()) {
      ROS_INFO("[%s] Success!", name_.c_str());

      flex_nav_common::FollowPathResult result;
      result.code = flex_nav_common::FollowPathResult::SUCCESS;
      result.pose = location.pose;

      fp_server_->setSucceeded(result, "Reached the goal!");
      running_ = false;
      return;
    }

    geometry_msgs::Twist cmd_vel;
    if (planner_->computeVelocityCommands(cmd_vel)) {
      geometry_msgs::TwistStamped result;
      result.header.stamp = ros::Time::now();
      result.header.frame_id = robot_frame_;
      result.twist = cmd_vel;

      vel_pub_.publish(result);

      flex_nav_common::FollowPathFeedback feedback;
      feedback.twist = cmd_vel;
      feedback.pose = location.pose;

      fp_server_->publishFeedback(feedback);
    } else {
      ROS_ERROR("[%s] Failed to get a twist from the local planner",
                name_.c_str());

      flex_nav_common::FollowPathResult abort;
      abort.code = flex_nav_common::FollowPathResult::FAILURE; // Planner failed
      abort.pose = location.pose;

      fp_server_->setAborted(abort, "Planner failed");
      running_ = false;
      return;
    }

    r.sleep();
  }

  costmap_->getRobotPose(pose);

  if (fp_server_->isPreemptRequested()) {
    ROS_WARN("[%s] Preempting goal...", name_.c_str());

    flex_nav_common::FollowPathResult abort;
    abort.code = flex_nav_common::FollowPathResult::PREEMPT; // Goal preempted
    abort.pose = location.pose;

    fp_server_->setPreempted(abort, "Goal preempted");
  } else {
    ROS_WARN("[%s] Canceling goal ...", name_.c_str());

    flex_nav_common::FollowPathResult abort;
    abort.code = flex_nav_common::FollowPathResult::PREEMPT; // Goal canceled
    abort.pose = location.pose;

    fp_server_->setPreempted(abort, "Goal canceled");
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
