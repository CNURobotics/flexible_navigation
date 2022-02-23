/*******************************************************************************
 *  Copyright (c) 2016
 *  Capable Humanitarian Robotics and Intelligent Systems Lab (CHRISLab)
 *  Christopher Newport University
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:

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

#include "rclcpp/rclcpp.hpp"
#include <flex_nav_controllers/follow_topic.h>
#include "nav_2d_utils/conversions.hpp"
#include "nav_2d_utils/tf_help.hpp"
#include "nav2_core/exceptions.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

namespace flex_nav {
  FollowTopic::FollowTopic()
      : nav2_util::LifecycleNode("follow_topic", "", true),
        progress_checker_loader_("nav2_core", "nav2_core::ProgressChecker"),
        default_progress_checker_id_{"progress_checker"},
        default_progress_checker_type_{"nav2_controller::SimpleProgressChecker"},
        goal_checker_loader_("nav2_core", "nav2_core::GoalChecker"),
        default_goal_checker_id_{"goal_checker"},
        default_goal_checker_type_{"nav2_controller::SimpleGoalChecker"},
        lp_loader_("nav2_core", "nav2_core::Controller"),
        default_id_{"FollowPath"},
        default_type_{"dwb_core::DWBLocalPlanner"},
        running_(false),
        name_("follow_topic") {
    using namespace std::placeholders;

    declare_parameter("controller_frequency", 20.0);
    declare_parameter("progress_checker_plugin", default_progress_checker_id_);
    declare_parameter("goal_checker_plugin", default_goal_checker_id_);
    declare_parameter("controller_plugin", default_id_);
    declare_parameter("min_x_velocity_threshold", rclcpp::ParameterValue(0.0001));
    declare_parameter("min_y_velocity_threshold", rclcpp::ParameterValue(0.0001));
    declare_parameter("min_theta_velocity_threshold", rclcpp::ParameterValue(0.0001));
    declare_parameter("velocity_publisher", "");
    declare_parameter("velocity_stamp_publisher", "");

    costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
      "local_costmap", std::string{get_namespace()}, "local_costmap");

    costmap_thread_ = std::make_unique<nav2_util::NodeThread>(costmap_ros_);
  }

  FollowTopic::~FollowTopic() {
    costmap_thread_.reset();
    odom_sub_.reset();
    controller_.reset();
  }

  nav2_util::CallbackReturn
  FollowTopic::on_configure(const rclcpp_lifecycle::State & state)
  {
    name_ = this->get_name();
    ft_server_ = std::make_unique<FollowTopicActionServer>(
      rclcpp_node_,
      name_,
      std::bind(&FollowTopic::execute, this));

    cc_server_ = std::make_unique<ClearCostmapActionServer>(
      rclcpp_node_,
      name_ + "/clear_costmap",
      std::bind(&FollowTopic::clear_costmap, this));

    auto node = shared_from_this();

    RCLCPP_INFO(get_logger(), "Configuring controller interface");

    get_parameter("progress_checker_plugin", progress_checker_id_);
    if (progress_checker_id_ == default_progress_checker_id_) {
      nav2_util::declare_parameter_if_not_declared(
        node, default_progress_checker_id_ + ".plugin",
        rclcpp::ParameterValue(default_progress_checker_type_));
    }
    get_parameter("goal_checker_plugin", goal_checker_id_);
    if (goal_checker_id_ == default_goal_checker_id_) {
      nav2_util::declare_parameter_if_not_declared(
        node, default_goal_checker_id_ + ".plugin",
        rclcpp::ParameterValue(default_goal_checker_type_));
    }

    get_parameter("controller_plugin", controller_id_);
    if (controller_id_ == default_id_) {
      nav2_util::declare_parameter_if_not_declared(node, default_id_ + ".plugin",
        rclcpp::ParameterValue(default_type_));
    }

    get_parameter("controller_frequency", controller_frequency_);
    get_parameter("min_x_velocity_threshold", min_x_velocity_threshold_);
    get_parameter("min_y_velocity_threshold", min_y_velocity_threshold_);
    get_parameter("min_theta_velocity_threshold", min_theta_velocity_threshold_);
    get_parameter("velocity_publisher", vel_publisher_name_);
    get_parameter("velocity_stamp_publisher", vel_stamp_publisher_name_);
    RCLCPP_INFO(get_logger(), "Controller frequency set to %.4fHz", controller_frequency_);

    costmap_ros_->on_configure(state);

    try {
      progress_checker_type_ = nav2_util::get_plugin_type_param(node, progress_checker_id_);
      progress_checker_ = progress_checker_loader_.createUniqueInstance(progress_checker_type_);
      RCLCPP_INFO(get_logger(), "Created progress_checker : %s of type %s",
        progress_checker_id_.c_str(), progress_checker_type_.c_str());
      progress_checker_->initialize(node, progress_checker_id_);
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(get_logger(), "Failed to create progress_checker. Exception: %s",
        ex.what());
      return nav2_util::CallbackReturn::FAILURE;
    }

    try {
      goal_checker_type_ = nav2_util::get_plugin_type_param(node, goal_checker_id_);
      goal_checker_ = goal_checker_loader_.createUniqueInstance(goal_checker_type_);
      RCLCPP_INFO(get_logger(), "Created goal_checker : %s of type %s",
        goal_checker_id_.c_str(), goal_checker_type_.c_str());
      goal_checker_->initialize(node, goal_checker_id_);
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(get_logger(), "Failed to create goal_checker. Exception: %s",
        ex.what());
      return nav2_util::CallbackReturn::FAILURE;
    }

    try {
      controller_type_ = nav2_util::get_plugin_type_param(node, controller_id_);
      controller_ = lp_loader_.createUniqueInstance(controller_type_);
      RCLCPP_INFO(get_logger(), "Created controller : %s of type %s",
        controller_id_.c_str(), controller_type_.c_str());
      controller_->configure(node, controller_id_, costmap_ros_->getTfBuffer(), costmap_ros_);
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(get_logger(), "Failed to create controller. Exception: %s",
        ex.what());
      return nav2_util::CallbackReturn::FAILURE;
    }

    odom_sub_ = std::make_unique<nav_2d_utils::OdomSubscriber>(node);

    if (vel_publisher_name_ != "") {
      vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>(vel_publisher_name_, 1);
    }

    if (vel_stamp_publisher_name_ != "") {
      vel_stamp_publisher_ = create_publisher<geometry_msgs::msg::TwistStamped>(vel_stamp_publisher_name_, 10);
    }

    if (vel_publisher_name_ == "" && vel_stamp_publisher_name_ == "") {
      vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
    }

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  FollowTopic::on_activate(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "Activating");

    costmap_ros_->on_activate(state);
    controller_->activate();
    ft_server_->activate();
    cc_server_->activate();

    if (vel_publisher_name_ != "") {
      vel_publisher_->on_activate();
    }

    if (vel_stamp_publisher_name_ != "") {
      vel_stamp_publisher_->on_activate();
    }

    if (vel_publisher_name_ == "" && vel_stamp_publisher_name_ == "") {
      vel_publisher_->on_activate();
    }

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  FollowTopic::on_deactivate(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "Deactivating");
    controller_->deactivate();
    ft_server_->deactivate();
    cc_server_->deactivate();
    costmap_ros_->on_deactivate(state);

    publishZeroVelocity();

    if (vel_publisher_name_ != "") {
      vel_publisher_->on_deactivate();
    }

    if (vel_stamp_publisher_name_ != "") {
      vel_stamp_publisher_->on_deactivate();
    }

    if (vel_publisher_name_ == "" && vel_stamp_publisher_name_ == "") {
      vel_publisher_->on_deactivate();
    }

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  FollowTopic::on_cleanup(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "Cleaning up");
    controller_->cleanup();
    costmap_ros_->on_cleanup(state);

    ft_server_.reset();
    cc_server_.reset();
    odom_sub_.reset();
    vel_publisher_.reset();
    vel_stamp_publisher_.reset();
    goal_checker_->reset();

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  FollowTopic::on_shutdown(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Shutting down");
    return nav2_util::CallbackReturn::SUCCESS;
  }

  void FollowTopic::setPlannerPath(const nav_msgs::msg::Path & path)
  {
    if (path.poses.empty()) {
      throw nav2_core::PlannerException("Invalid path, Path is empty.");
    }
    controller_->setPlan(path);

    auto end_pose = path.poses.back();
    end_pose.header.frame_id = path.header.frame_id;
    rclcpp::Duration tolerance(costmap_ros_->getTransformTolerance() * 1e9);
    nav_2d_utils::transformPose(costmap_ros_->getTfBuffer(), costmap_ros_->getGlobalFrameID(),
      end_pose, end_pose, tolerance);
    goal_checker_->reset();

    RCLCPP_DEBUG(get_logger(), "Path end point is (%.2f, %.2f)",
      end_pose.pose.position.x, end_pose.pose.position.y);
    end_pose_ = end_pose.pose;
  }

  void FollowTopic::publishZeroVelocity()
  {
    geometry_msgs::msg::TwistStamped velocity;
    velocity.twist.angular.x = 0;
    velocity.twist.angular.y = 0;
    velocity.twist.angular.z = 0;
    velocity.twist.linear.x = 0;
    velocity.twist.linear.y = 0;
    velocity.twist.linear.z = 0;
    velocity.header.frame_id = costmap_ros_->getBaseFrameID();
    velocity.header.stamp = now();

    if (vel_publisher_name_ != "") {
      vel_publisher_->publish(velocity.twist);
    }

    if (vel_stamp_publisher_name_ != "") {
      vel_stamp_publisher_->publish(velocity);
    }

    if (vel_publisher_name_ == "" && vel_stamp_publisher_name_ == "") {
      vel_publisher_->publish(velocity.twist);
    }
  }

  void FollowTopic::computeAndPublishVelocity()
  {
    geometry_msgs::msg::PoseStamped pose;

    if (!getRobotPose(pose)) {
      throw nav2_core::PlannerException("Failed to obtain robot pose");
    }

    if (!progress_checker_->check(pose)) {
      throw nav2_core::PlannerException("Failed to make progress");
    }

    nav_2d_msgs::msg::Twist2D twist = odom_sub_->getTwist();

    auto cmd_vel_2d = controller_->computeVelocityCommands(pose, nav_2d_utils::twist2Dto3D(twist));
    publishVelocity(cmd_vel_2d, pose);
  }

  void FollowTopic::publishVelocity(const geometry_msgs::msg::TwistStamped & velocity, geometry_msgs::msg::PoseStamped robotPose)
  {
    auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>(velocity.twist);
    auto cmd_vel_stamp = std::make_unique<geometry_msgs::msg::TwistStamped>(velocity);

    if (vel_stamp_publisher_name_ != "") {
      if (vel_stamp_publisher_->is_activated() && this->count_subscribers(vel_stamp_publisher_->get_topic_name()) > 0)
      {
        vel_stamp_publisher_->publish(std::move(cmd_vel_stamp));
      }
    }

    if (vel_publisher_name_ != "" || (vel_publisher_name_ == "" && vel_stamp_publisher_name_ == "")) {
      if (vel_publisher_->is_activated() && this->count_subscribers(vel_publisher_->get_topic_name()) > 0)
      {
        vel_publisher_->publish(std::move(cmd_vel));
      }
    }
  }

  bool FollowTopic::isGoalReached()
  {
    geometry_msgs::msg::PoseStamped pose;

    if (!getRobotPose(pose)) {
      return false;
    }

    nav_2d_msgs::msg::Twist2D twist = odom_sub_->getTwist();
    geometry_msgs::msg::Twist velocity = nav_2d_utils::twist2Dto3D(twist);
    return goal_checker_->isGoalReached(pose.pose, end_pose_, velocity);
  }

  bool FollowTopic::getRobotPose(geometry_msgs::msg::PoseStamped & pose)
  {
    geometry_msgs::msg::PoseStamped current_pose;
    if (!costmap_ros_->getRobotPose(current_pose)) {
      return false;
    }
    pose = current_pose;
    return true;
  }

  void FollowTopic::execute() {
    auto goal = ft_server_->get_current_goal();

    rclcpp::Rate r(controller_frequency_);
    geometry_msgs::msg::PoseStamped location;
    geometry_msgs::msg::PoseStamped pose;
    geometry_msgs::msg::Twist cmd_vel;
    geometry_msgs::msg::TwistStamped cur_twist;

    std::shared_ptr<flex_nav_common::action::FollowTopic::Result> result =
      std::make_shared<flex_nav_common::action::FollowTopic::Result>();

    auto feedback = std::make_shared<FollowTopicAction::Feedback>();

    while (running_) {
      RCLCPP_WARN(get_logger(), "[%s] Waiting for lock", name_.c_str());
      r.sleep();
    }

    current_path_ = nav_msgs::msg::Path();
    latest_path_ = nav_msgs::msg::Path();

    RCLCPP_INFO(get_logger(), "[%s] Attempting to listen to topic: %s",
                name_.c_str(), goal->topic.data.c_str());

    bool good(false);

    try {
      sub_ = create_subscription<nav_msgs::msg::Path>(
        goal->topic.data,
        rclcpp::SystemDefaultsQoS(),
        std::bind(&FollowTopic::topic_cb, this, std::placeholders::_1));

      RCLCPP_INFO(get_logger(), "[%s] Success!", name_.c_str());
      good = true;
    }
    catch(std::exception& e) {
      RCLCPP_WARN(get_logger(), "Unable to create subscription [%s]");
      good = false;
    }

    // This is not good
    if (!good) {
      RCLCPP_ERROR(get_logger(), "[%s] Desired topic does not publish a nav_msgs/Path",
        name_.c_str());

      result->code = flex_nav_common::action::FollowTopic::Result::FAILURE;
      ft_server_->terminate_current(result);
      sub_.reset();
      current_path_ = nav_msgs::msg::Path();
      latest_path_ = nav_msgs::msg::Path();
      return;
    }

    // Wait for a path to be received
    while ((latest_path_.poses.empty()) && rclcpp::ok() && !ft_server_->is_cancel_requested()) {
      r.sleep();
    }

    running_ = true;
    while (running_ && rclcpp::ok() && !ft_server_->is_cancel_requested()) {
      current_path_ = latest_path_;

      if (current_path_.poses.empty()) {
        RCLCPP_WARN(get_logger(), "Current plan to goal is empty");

        costmap_ros_->getRobotPose(pose);

        // Could not set plan
        result->code = flex_nav_common::action::FollowTopic::Result::FAILURE;
        result->pose = location.pose;

        ft_server_->terminate_current(result);
        sub_.reset();
        current_path_ = nav_msgs::msg::Path();
        latest_path_ = nav_msgs::msg::Path();
        return;
      }

      // Send the goal to the planner
      setPlannerPath(current_path_);
      progress_checker_->reset();

      // This is where the actual work gets done
      while(current_path_.header.stamp == latest_path_.header.stamp && running_ &&
        rclcpp::ok() && !ft_server_->is_cancel_requested()) {

          RCLCPP_DEBUG(get_logger(), "[%s] Generating path from path: #%u", name_.c_str(),
            current_path_.header);

          if (isGoalReached()) {
            RCLCPP_INFO(get_logger(), "Reached goal - Success!");
            publishZeroVelocity();

            result->code = flex_nav_common::action::FollowTopic::Result::SUCCESS;
            result->pose = location.pose;

            ft_server_->succeeded_current(result);
            sub_.reset();
            current_path_ = nav_msgs::msg::Path();
            latest_path_ = nav_msgs::msg::Path();
            running_ = false;
            return;
          }

          try {
            computeAndPublishVelocity();

            feedback->path = current_path_;
            feedback->pose = location.pose;
            ft_server_->publish_feedback(feedback);
          }
          catch (std::exception& e) {
            RCLCPP_ERROR(get_logger(), "[%s] Failed to get a plan from the local planner",
              name_.c_str());

            result->code = flex_nav_common::action::FollowTopic::Result::FAILURE; // Planner failed
            result->pose = location.pose;

            ft_server_->terminate_current(result);
            sub_.reset();
            current_path_ = nav_msgs::msg::Path();
            latest_path_ = nav_msgs::msg::Path();
            running_ = false;
            publishZeroVelocity();
            return;
          }

          r.sleep();
      }
      r.sleep();
    }

    publishZeroVelocity();
    costmap_ros_->getRobotPose(pose);

    if (ft_server_->is_cancel_requested()) {
      RCLCPP_WARN(get_logger(), "[%s] Canceling goal...", name_.c_str());
      result->code = flex_nav_common::action::FollowTopic::Result::CANCEL;
      result->pose = location.pose;
      ft_server_->terminate_current(result);
    } else {
      result->code = flex_nav_common::action::FollowTopic::Result::SUCCESS;
      result->pose = location.pose;
      ft_server_->succeeded_current(result);
    }

    running_ = false;
    sub_.reset();
    current_path_ = nav_msgs::msg::Path();
    latest_path_ = nav_msgs::msg::Path();
  }

  void FollowTopic::topic_cb(const nav_msgs::msg::Path::SharedPtr data) {
    RCLCPP_DEBUG(get_logger(), "[%s] Recieved a new path with %lu points: #%u",
      name_.c_str(),
      data->poses.size(), data->header);

    latest_path_ = *data;
  }

  void FollowTopic::clear_costmap() {
    auto goal = cc_server_->get_current_goal();
    costmap_ros_->resetLayers();
    std::shared_ptr<flex_nav_common::action::ClearCostmap::Result> result =
      std::make_shared<flex_nav_common::action::ClearCostmap::Result>();
    result->code = flex_nav_common::action::ClearCostmap::Result::SUCCESS;
    cc_server_->succeeded_current(result);
  }
}
