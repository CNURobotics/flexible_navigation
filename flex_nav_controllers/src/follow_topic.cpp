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

#include <flex_nav_controllers/follow_topic.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

namespace flex_nav {
FollowTopic::FollowTopic(tf2_ros::Buffer &tf)
    : tf_(tf), ft_server_(NULL), costmap_(NULL),
      loader_("nav_core", "nav_core::BaseLocalPlanner"), running_(false),
      name_(ros::this_node::getName()) {
  ros::NodeHandle private_nh("~");
  ros::NodeHandle nh;

  ft_server_ = new FollowTopicActionServer(
      nh, name_, boost::bind(&FollowTopic::execute, this, _1), false);
  cc_server_ = new ClearCostmapActionServer(
      nh, name_ + "/clear_costmap",
      boost::bind(&FollowTopic::clear_costmap, this, _1), false);

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
  ft_server_->start();
  cc_server_->start();
  ros::spin();
}

FollowTopic::~FollowTopic() {
  ft_server_->shutdown();
  cc_server_->shutdown();
}

void FollowTopic::execute(
    const flex_nav_common::FollowTopicGoalConstPtr &goal) {
  ros::NodeHandle n;
  ros::Rate r(controller_frequency_);
  geometry_msgs::PoseStamped location;
  geometry_msgs::PoseStamped pose;
  flex_nav_common::FollowTopicResult result;
  flex_nav_common::FollowTopicFeedback feedback;
  geometry_msgs::Twist cmd_vel;
  geometry_msgs::TwistStamped cur_twist;

  while (running_) {
    ROS_WARN_THROTTLE(0.25, "[%s] Waiting for lock", name_.c_str());
    r.sleep();
  }

  current_path_.reset();
  latest_path_.reset();

  ROS_INFO("[%s] Attempting to listen to topic: %s", name_.c_str(),
           goal->topic.data.c_str());

  bool good(false);

  ros::master::V_TopicInfo master_topics;
  ros::master::getTopics(master_topics);

  for (ros::master::V_TopicInfo::iterator it = master_topics.begin();
       it != master_topics.end(); it++) {
    const ros::master::TopicInfo &info = *it;

    if (info.name.find(goal->topic.data) != std::string::npos &&
        !info.datatype.compare("nav_msgs/Path")) {
      sub_ = n.subscribe(goal->topic.data, 1, &FollowTopic::topic_cb, this);

      ROS_INFO("[%s] Success!", name_.c_str());
      good = true;
      break;
    }
  }

  // This is not good
  if (!good) {
    ROS_ERROR("[%s] Desired topic does not publish a nav_msgs/Path",
              name_.c_str());
    ft_server_->setAborted();
    sub_.shutdown();
    current_path_.reset();
    latest_path_.reset();
    return;
  }

  // Wait for a path to be received
  while ((!latest_path_ || latest_path_->poses.size() == 0) && n.ok() &&
         !ft_server_->isNewGoalAvailable() &&
         !ft_server_->isPreemptRequested()) {
    r.sleep();
  }

  running_ = true;
  while (running_ && n.ok() && !ft_server_->isNewGoalAvailable() &&
         !ft_server_->isPreemptRequested()) {
    current_path_ = latest_path_;

    // Send the goal to the planner
    if (!planner_->setPlan(current_path_->poses)) {
      costmap_->getRobotPose(pose);

      result.code =
          flex_nav_common::FollowTopicResult::FAILURE; // Could not set plan
      result.pose = location.pose;

      ft_server_->setAborted(result, "Could not set plan");
      sub_.shutdown();
      current_path_.reset();
      latest_path_.reset();
      return;
    }

    // This is where the actual work gets done
    do {
      costmap_->getRobotPose(pose);

      ROS_DEBUG("[%s] Generating path from path: #%u", name_.c_str(),
                current_path_->header.seq);

      if (planner_->isGoalReached()) {
        ROS_INFO("Reached goal");
        result.code = flex_nav_common::FollowTopicResult::SUCCESS;
        result.pose = location.pose;

        ft_server_->setSucceeded(result, "Reached the goal!");
        sub_.shutdown();
        current_path_.reset();
        latest_path_.reset();
        running_ = false;
        return;
      }

      if (planner_->computeVelocityCommands(cmd_vel)) {
        cur_twist.header.stamp = ros::Time::now();
        cur_twist.header.frame_id = robot_frame_;
        cur_twist.twist = cmd_vel;

        vel_pub_.publish(cur_twist);

        // feedback.twist = cmd_vel;
        feedback.pose = location.pose;

        ft_server_->publishFeedback(feedback);
      } else {
        ROS_ERROR("[%s] Failed to get a plan from the local planner",
                  name_.c_str());

        result.code =
            flex_nav_common::FollowTopicResult::FAILURE; // Planner failed
        result.pose = location.pose;

        ft_server_->setAborted(result, "Planner failed");
        sub_.shutdown();
        current_path_.reset();
        latest_path_.reset();
        running_ = false;
        return;
      }

      r.sleep();
    } while (current_path_->header.stamp == latest_path_->header.stamp &&
             running_ && n.ok() && !ft_server_->isNewGoalAvailable() &&
             !ft_server_->isPreemptRequested());

    r.sleep();
  }

  costmap_->getRobotPose(pose);

  if (ft_server_->isPreemptRequested()) {
    ROS_WARN("[%s] Preempting goal...", name_.c_str());

    result.code = flex_nav_common::FollowTopicResult::PREEMPT;
    result.pose = location.pose;

    ft_server_->setPreempted(result, "Goal preempted");
  } else {
    ROS_WARN("[%s] Canceling goal ...", name_.c_str());

    result.code = flex_nav_common::FollowTopicResult::PREEMPT; // Goal canceled
    result.pose = location.pose;

    ft_server_->setPreempted(result, "Goal canceled");
  }
  running_ = false;
  sub_.shutdown();
  current_path_.reset();
  latest_path_.reset();
}

void FollowTopic::topic_cb(const nav_msgs::PathConstPtr &data) {
  ROS_DEBUG("[%s] Recieved a new path with %lu points: #%u", name_.c_str(),
            data->poses.size(), data->header.seq);

  latest_path_ = data;
}

void FollowTopic::clear_costmap(
    const flex_nav_common::ClearCostmapGoalConstPtr &goal) {
  costmap_->resetLayers();

  flex_nav_common::ClearCostmapResult result;
  result.code = flex_nav_common::ClearCostmapResult::SUCCESS;
  cc_server_->setSucceeded(result, "Success");
}
}
