/*******************************************************************************
 *  Copyright (c) 2016-2022
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
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "nav_2d_utils/conversions.hpp"
#include "nav_2d_utils/tf_help.hpp"
#include "nav2_core/exceptions.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include "rclcpp/rclcpp.hpp"
#include <exception>

namespace flex_nav {
  FollowPath::FollowPath(const rclcpp::NodeOptions & options)
      : nav2_util::LifecycleNode("follow_path", "", options),
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
        name_("follow_path")
        {
    using namespace std::placeholders;

    declare_parameter("controller_frequency", rclcpp::ParameterValue(20.0));
    declare_parameter("progress_checker_plugin", rclcpp::ParameterValue(default_progress_checker_id_));
    declare_parameter("goal_checker_plugin", rclcpp::ParameterValue(default_goal_checker_id_));
    declare_parameter("controller_plugin", rclcpp::ParameterValue(default_id_));
    declare_parameter("min_x_velocity_threshold", rclcpp::ParameterValue(0.0001));
    declare_parameter("min_y_velocity_threshold", rclcpp::ParameterValue(0.0001));
    declare_parameter("min_theta_velocity_threshold", rclcpp::ParameterValue(0.0001));
    declare_parameter("velocity_publisher", rclcpp::ParameterValue(""));
    declare_parameter("velocity_stamp_publisher", rclcpp::ParameterValue(""));

    costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
      "local_costmap", std::string{get_namespace()}, "local_costmap");

    costmap_thread_ = std::make_unique<nav2_util::NodeThread>(costmap_ros_);
  }

  FollowPath::~FollowPath() {
    costmap_thread_.reset();
    odom_sub_.reset();
    controller_.reset();
  }

  nav2_util::CallbackReturn
  FollowPath::on_configure(const rclcpp_lifecycle::State & state)
  {
    name_ = this->get_name();
    RCLCPP_INFO(get_logger(), "Configuring %s", name_.c_str());

    fp_server_ = std::make_unique<FollowPathActionServer>(
      get_node_base_interface(),
      get_node_clock_interface(),
      get_node_logging_interface(),
      get_node_waitables_interface(),
      name_,
      std::bind(&FollowPath::execute, this));



    RCLCPP_DEBUG(get_logger(), "Configuring CM action server for %s", name_.c_str());
    cc_server_ = std::make_unique<ClearCostmapActionServer>(
      get_node_base_interface(),
      get_node_clock_interface(),
      get_node_logging_interface(),
      get_node_waitables_interface(),
      name_ + "/clear_costmap",
      std::bind(&FollowPath::clear_costmap, this));

    auto node = shared_from_this();

    RCLCPP_DEBUG(get_logger(), "  Get parameters for controller interface %s", name_.c_str());

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

    RCLCPP_DEBUG(get_logger(), "  Configure costmap for %s", name_.c_str());
    costmap_ros_->on_configure(state);

    RCLCPP_DEBUG(get_logger(), "  Set up the plugings for controller interface %s", name_.c_str());
    try {
      progress_checker_type_ = nav2_util::get_plugin_type_param(node, progress_checker_id_);
      progress_checker_ = progress_checker_loader_.createUniqueInstance(progress_checker_type_);
      RCLCPP_DEBUG(get_logger(), "Created progress_checker : %s of type %s",
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
      RCLCPP_DEBUG(get_logger(), "Created goal_checker : %s of type %s",
          goal_checker_id_.c_str(), goal_checker_type_.c_str());
      goal_checker_->initialize(node, goal_checker_id_, costmap_ros_);
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(get_logger(), "Failed to create goal_checker. Exception: %s",
        ex.what());
      return nav2_util::CallbackReturn::FAILURE;
    }

    try {
      controller_type_ = nav2_util::get_plugin_type_param(node, controller_id_);
      controller_ = lp_loader_.createUniqueInstance(controller_type_);
      RCLCPP_DEBUG(get_logger(), "Created controller : %s of type %s",
        controller_id_.c_str(), controller_type_.c_str());
      controller_->configure(node, controller_id_, costmap_ros_->getTfBuffer(), costmap_ros_);
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(get_logger(), "Failed to create controller. Exception: %s",
        ex.what());
      return nav2_util::CallbackReturn::FAILURE;
    }

    RCLCPP_DEBUG(get_logger(), "  Set up odom sub for %s", name_.c_str());
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

    RCLCPP_DEBUG(get_logger(), " Configure success for %s", name_.c_str());
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  FollowPath::on_activate(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "Activating %s", name_.c_str());
    controller_->activate();

    RCLCPP_DEBUG(get_logger(), "   Activating costmap for  %s", name_.c_str());
    costmap_ros_->on_activate(state);
    fp_server_->activate();
    cc_server_->activate();

    RCLCPP_DEBUG(get_logger(), "Activating publishers for %s", name_.c_str());
    if (vel_publisher_name_ != "") {
      vel_publisher_->on_activate();
    }

    if (vel_stamp_publisher_name_ != "") {
      vel_stamp_publisher_->on_activate();
    }

    if (vel_publisher_name_ == "" && vel_stamp_publisher_name_ == "") {
      vel_publisher_->on_activate();
    }

    // create bond connection with nav2_util::LifeCycle manager
    createBond();

    RCLCPP_DEBUG(get_logger(), "Activated SUCCESS for %s", name_.c_str());
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  FollowPath::on_deactivate(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "Deactivating %s", name_.c_str());
    controller_->deactivate();
    fp_server_->deactivate();
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

    // destroy bond connection with nav2_util::LifeCycle manager
    destroyBond();

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  FollowPath::on_cleanup(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "Cleaning up %s", name_.c_str());
    controller_->cleanup();
    costmap_ros_->on_cleanup(state);

    fp_server_.reset();
    cc_server_.reset();
    odom_sub_.reset();
    vel_publisher_.reset();
    vel_stamp_publisher_.reset();
    goal_checker_->reset();

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  FollowPath::on_shutdown(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Shutting down %s", name_.c_str());
    return nav2_util::CallbackReturn::SUCCESS;
  }

  void FollowPath::publishZeroVelocity()
  {
    geometry_msgs::msg::TwistStamped velocity = geometry_msgs::msg::TwistStamped();
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

  void FollowPath::setPlannerPath(const nav_msgs::msg::Path & path)
  {
    if (path.poses.empty()) {
      throw nav2_core::PlannerException("Invalid path, Path is empty.");
    }
    controller_->setPlan(path);

    auto end_pose = path.poses.back();
    end_pose.header.frame_id = path.header.frame_id;
    rclcpp::Duration tolerance = rclcpp::Duration::from_nanoseconds(costmap_ros_->getTransformTolerance() * 1e9);

    nav_2d_utils::transformPose(
      costmap_ros_->getTfBuffer(), costmap_ros_->getGlobalFrameID(),
      end_pose, end_pose, tolerance);
    goal_checker_->reset();

    RCLCPP_DEBUG(get_logger(), "Path end point is (%.2f, %.2f)",
      end_pose.pose.position.x, end_pose.pose.position.y);
    end_pose_ = end_pose.pose;
  }

  void FollowPath::computeAndPublishVelocity()
  {
    geometry_msgs::msg::PoseStamped pose;

    if (!getRobotPose(pose)) {
      throw nav2_core::PlannerException("Failed to obtain robot pose");
    }

    if (!progress_checker_->check(pose)) {
      throw nav2_core::PlannerException("Failed to make progress");
    }

    nav_2d_msgs::msg::Twist2D twist = getThresholdedTwist(odom_sub_->getTwist());

    auto cmd_vel_2d =
      controller_->computeVelocityCommands(pose,
                                           nav_2d_utils::twist2Dto3D(twist),
                                           goal_checker_.get());

    publishVelocity(cmd_vel_2d, pose);
  }

  void FollowPath::updateGlobalPath()
  {
    if (fp_server_->is_cancel_requested()) {
      RCLCPP_INFO(get_logger(), "Passing new path to controller.");
      auto goal = fp_server_->accept_pending_goal();
      std::string current_controller;
      setPlannerPath(goal->path);
    }
  }

  void FollowPath::publishVelocity(const geometry_msgs::msg::TwistStamped & velocity, geometry_msgs::msg::PoseStamped robotPose)
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

    auto feedback = std::make_shared<FollowPathAction::Feedback>();

    feedback->twist = velocity.twist;
    feedback->pose = robotPose.pose;

    fp_server_->publish_feedback(feedback);
  }

  bool FollowPath::isGoalReached()
  {
    geometry_msgs::msg::PoseStamped pose;

    if (!getRobotPose(pose)) {
      return false;
    }

    nav_2d_msgs::msg::Twist2D twist = odom_sub_->getTwist();
    geometry_msgs::msg::Twist velocity = nav_2d_utils::twist2Dto3D(twist);
    return goal_checker_->isGoalReached(pose.pose, end_pose_, velocity);
  }

  bool FollowPath::getRobotPose(geometry_msgs::msg::PoseStamped & pose)
  {
    geometry_msgs::msg::PoseStamped current_pose;
    if (!costmap_ros_->getRobotPose(current_pose)) {
      return false;
    }
    pose = current_pose;
    return true;
  }

  void FollowPath::execute() {
    auto goal = fp_server_->get_current_goal();
    geometry_msgs::msg::PoseStamped location;
    rclcpp::Rate r(controller_frequency_);
    try {
      while (running_) {
        RCLCPP_WARN(get_logger(), "[%s] Waiting for lock", name_.c_str());
        r.sleep();
      }

      // If invalid path then terminate_current goal
      if (goal->path.poses.empty()) {
        RCLCPP_WARN(get_logger(), "Got path with no poses");
        std::shared_ptr<flex_nav_common::action::FollowPath::Result> abort =
          std::make_shared<flex_nav_common::action::FollowPath::Result>();

        abort->code = flex_nav_common::action::FollowPath::Result::FAILURE; // Could not set plan

        costmap_ros_->getRobotPose(location);
        abort->pose = location.pose;

        fp_server_->terminate_current(abort);
        return;
      }

      // Plan path to goal
      setPlannerPath(goal->path);
      progress_checker_->reset();

      running_ = true;
      while (running_ && rclcpp::ok()) {
        if (fp_server_ == nullptr || !fp_server_->is_server_active()) {
            RCLCPP_INFO(get_logger(), "Action server unavailable or inactive. Stopping.");
            std::shared_ptr<flex_nav_common::action::FollowPath::Result> abort =
              std::make_shared<flex_nav_common::action::FollowPath::Result>();
            abort->code = flex_nav_common::action::FollowPath::Result::FAILURE; // Could not set plan
            costmap_ros_->getRobotPose(location);
            abort->pose = location.pose;
            fp_server_->terminate_current(abort);
            running_ = false;
            return;
        }

        if (fp_server_->is_cancel_requested()) {
          RCLCPP_INFO(get_logger(), "Goal was canceled. Stopping the robot.");
          fp_server_->terminate_all();
          publishZeroVelocity();

          std::shared_ptr<flex_nav_common::action::FollowPath::Result> abort =
            std::make_shared<flex_nav_common::action::FollowPath::Result>();

          abort->code = flex_nav_common::action::FollowPath::Result::CANCEL; // Could not set plan

          costmap_ros_->getRobotPose(location);
          abort->pose = location.pose;

          fp_server_->terminate_current(abort);

          running_ = false;
          return;
        }

        updateGlobalPath();

        computeAndPublishVelocity();

        if (isGoalReached()) {
          RCLCPP_INFO(get_logger(), "[%s] Success!", name_.c_str());
          std::shared_ptr<flex_nav_common::action::FollowPath::Result> result =
            std::make_shared<flex_nav_common::action::FollowPath::Result>();
          result->code = flex_nav_common::action::FollowPath::Result::SUCCESS;

          costmap_ros_->getRobotPose(location);
          result->pose = location.pose;

          fp_server_->succeeded_current(result);
          running_ = false;

          publishZeroVelocity();
          break;
        }

        if (!r.sleep()) {
          RCLCPP_WARN(get_logger(), "Control loop missed its desired rate of %.4fHz",
            controller_frequency_);
        }
      }
    }
    catch (nav2_core::PlannerException & e) {
      RCLCPP_ERROR(get_logger(), e.what());
      publishZeroVelocity();
      std::shared_ptr<flex_nav_common::action::FollowPath::Result> abort =
        std::make_shared<flex_nav_common::action::FollowPath::Result>();

      abort->code = flex_nav_common::action::FollowPath::Result::FAILURE; // Could not set plan
      costmap_ros_->getRobotPose(location);
      abort->pose = location.pose;

      fp_server_->terminate_current(abort);
      running_ = false;
      return;
    }
  }

  void FollowPath::clear_costmap() {
    auto goal = cc_server_->get_current_goal();
    costmap_ros_->resetLayers();
    std::shared_ptr<flex_nav_common::action::ClearCostmap::Result> result =
      std::make_shared<flex_nav_common::action::ClearCostmap::Result>();
    result->code = flex_nav_common::action::ClearCostmap::Result::SUCCESS;
    cc_server_->succeeded_current(result);
  }
}
