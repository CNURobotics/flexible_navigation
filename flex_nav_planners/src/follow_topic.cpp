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

#include <fstream>
#include <memory>
#include <streambuf>
#include <string>
#include <utility>
#include <vector>
#include <set>
#include <exception>
#include <flex_nav_planners/follow_common.h>
#include <flex_nav_planners/follow_topic.h>
#include <geometry_msgs/msg/twist.h>

using std::placeholders::_1;

namespace flex_nav {
  FollowTopic::FollowTopic()
      : nav2_util::LifecycleNode("follow_topic", "", true),
        ft_loader_("nav2_core", "nav2_core::GlobalPlanner"),
        default_id_{"GridBased"},
        default_type_{"nav2_navfn_planner/NavfnPlanner"},
        costmap_(nullptr),
        name_("follow_topic"),
        running_(false) {
    using namespace std::placeholders;

    declare_parameter("planner_plugin", rclcpp::ParameterValue(default_id_));
    declare_parameter("distance_threshold", rclcpp::ParameterValue(5.0));
    declare_parameter("expected_planner_frequency", rclcpp::ParameterValue(1.0));
    declare_parameter("costmap_name", rclcpp::ParameterValue("low_middle_costmap"));

    get_parameter("costmap_name", costmap_name_);

    costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
      costmap_name_, std::string{get_namespace()}, costmap_name_);

    costmap_thread_ = std::make_unique<nav2_util::NodeThread>(costmap_ros_);
  }

  FollowTopic::~FollowTopic() {
    planner_.reset();
    costmap_thread_.reset();
  }

  nav2_util::CallbackReturn
  FollowTopic::on_configure(const rclcpp_lifecycle::State & state)
  {
    name_ = this->get_name();
    RCLCPP_INFO(get_logger(), "Configuring %s", name_.c_str());
    ft_server_ = std::make_unique<FollowTopicActionServer>(
      rclcpp_node_,
      name_,
      std::bind(&FollowTopic::execute, this));

    cc_server_ = std::make_unique<ClearCostmapActionServer>(
      rclcpp_node_,
      name_ + "/clear_costmap",
      std::bind(&FollowTopic::clear_costmap, this));

    get_parameter("expected_planner_frequency", expected_planner_frequency_);
    get_parameter("distance_threshold", distance_threshold_);

    costmap_ros_->on_configure(state);
    costmap_ = costmap_ros_->getCostmap();

    RCLCPP_DEBUG(get_logger(), "Costmap size: %d,%d",
      costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());

    tf_ = costmap_ros_->getTfBuffer();

    get_parameter("planner_plugin", planner_id_);
    if (planner_id_ == default_id_) {
      declare_parameter(default_id_ + ".plugin", rclcpp::ParameterValue(default_type_));
    }

    auto node = shared_from_this();

    try {
      planner_type_ = nav2_util::get_plugin_type_param(node, planner_id_);
      planner_ = ft_loader_.createUniqueInstance(planner_type_);
      RCLCPP_INFO(get_logger(), "Created global planner plugin %s of type %s",
        planner_id_.c_str(), planner_type_.c_str());
      planner_->configure(node, planner_id_, tf_, costmap_ros_);
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(get_logger(), "Failed to create global planner. Exception: %s",
        ex.what());
      return nav2_util::CallbackReturn::FAILURE;
    }

    double expected_planner_frequency;
    get_parameter("expected_planner_frequency", expected_planner_frequency);
    if (expected_planner_frequency > 0) {
      max_planner_duration_ = 1 / expected_planner_frequency;
    } else {
      RCLCPP_WARN(get_logger(),
        "The expected planner frequency parameter is %.4f Hz. The value should to be greater"
        " than 0.0 to turn on duration overrrun warning messages", expected_planner_frequency);
      max_planner_duration_ = 0.0;
    }

    plan_publisher_ = create_publisher<nav_msgs::msg::Path>(name_ + "/plan", 1);

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  FollowTopic::on_activate(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "Activating %s", name_.c_str());
    plan_publisher_->on_activate();
    ft_server_->activate();
    cc_server_->activate();
    costmap_ros_->on_activate(state);
    planner_->activate();

    // create bond connection with nav2_util::LifeCycle manager
    //Galactic createBond();

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  FollowTopic::on_deactivate(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "Deactivating %s", name_.c_str());
    plan_publisher_->on_deactivate();
    ft_server_->deactivate();
    cc_server_->deactivate();
    costmap_ros_->on_deactivate(state);
    planner_->deactivate();

    // destroy bond connection with nav2_util::LifeCycle manager
    //Galactic destroyBond();

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  FollowTopic::on_cleanup(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "Cleaning up %s", name_.c_str());
    plan_publisher_.reset();
    ft_server_.reset();
    cc_server_.reset();
    tf_.reset();
    costmap_ros_->on_cleanup(state);
    planner_->cleanup();
    costmap_ = nullptr;

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  FollowTopic::on_shutdown(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Shutting down %s", name_.c_str());
    return nav2_util::CallbackReturn::SUCCESS;
  }

  void FollowTopic::execute() {
    auto goal = ft_server_->get_current_goal();
    rclcpp::Rate r(expected_planner_frequency_);
    current_path_ = nav_msgs::msg::Path();
    latest_path_ = nav_msgs::msg::Path();

    bool good(false);
    std::shared_ptr<flex_nav_common::action::FollowTopic::Result> result =
      std::make_shared<flex_nav_common::action::FollowTopic::Result>();

    while (running_) {
      RCLCPP_WARN(get_logger(), "[%s] Waiting for lock", name_.c_str());
      r.sleep();
    }

    RCLCPP_INFO(get_logger(), "[%s] Attempting to listen to topic: %s",
      name_.c_str(), goal->topic.data.c_str());

    try {
      sub_ = create_subscription<nav_msgs::msg::Path>(
        goal->topic.data,
        rclcpp::SystemDefaultsQoS(),
        std::bind(&FollowTopic::topic_cb, this, std::placeholders::_1));

      RCLCPP_INFO(get_logger(), "[%s] Success!", name_.c_str());
      good = true;
    }
    catch(std::exception& e) {
      RCLCPP_WARN(get_logger(), "Unable to create subscription [%s]", name_.c_str());
      good = false;
    }

    // This is not good
    if (!good) {
      RCLCPP_ERROR(get_logger(), "[%s] Desired topic does not publish a nav_msgs/msg/Path",
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
        RCLCPP_ERROR(get_logger(), "[%s] The path is empty!", name_.c_str());
        return;
      }

      if (current_path_.header.frame_id == "") {
        RCLCPP_ERROR(get_logger(), "[%s] The frame_id is empty!", name_.c_str());
        return;
      }

      geometry_msgs::msg::PoseStamped start_pose;
      geometry_msgs::msg::PoseStamped goal_pose = current_path_.poses.back();
      auto feedback = std::make_shared<FollowTopicAction::Feedback>();

      // Get the current pose of the robot
      if (!costmap_ros_->getRobotPose(start_pose)) {
        ft_server_->terminate_current();
        return;
      }

      result->pose = start_pose.pose;

      RCLCPP_DEBUG(get_logger(), "[%s] Generating path from path", name_.c_str());

      nav2_costmap_2d::Costmap2D *costmap = costmap_ros_->getCostmap();
      double threshold = distance_threshold_ * costmap->getResolution();
      if (distanceSquared(start_pose, goal_pose) <= threshold * threshold) {
        RCLCPP_INFO(get_logger(), "[%s] Reached goal - Success!", name_.c_str());
        result->code = flex_nav_common::action::FollowTopic::Result::SUCCESS;
        ft_server_->succeeded_current(result);
        running_ = false;
        return;
      }

      feedback->pose = goal_pose.pose;
      ft_server_->publish_feedback(feedback);

      nav_msgs::msg::Path path = planner_->createPlan(start_pose, goal_pose);
      if (!path.poses.empty()) {
        plan_publisher_->publish(std::move(path));
      }
      else {
        // empty plan
        RCLCPP_WARN(get_logger(), "[%s] Empty path - abort goal!", name_.c_str());
        result->code = flex_nav_common::action::FollowTopic::Result::FAILURE;
        ft_server_->terminate_current(result);
        running_ = false;
        return;
      }

      r.sleep();
    }

    if (ft_server_->is_cancel_requested()) {
      RCLCPP_WARN(get_logger(), "[%s] Preempting goal...", name_.c_str());
      result->code = flex_nav_common::action::FollowTopic::Result::CANCEL;
    } else {
      RCLCPP_ERROR(get_logger(), "[%s] Failed goal...", name_.c_str());
      result->code = flex_nav_common::action::FollowTopic::Result::FAILURE;
    }

    ft_server_->terminate_current(result);
    running_ = false;
  }

  void FollowTopic::topic_cb(const nav_msgs::msg::Path::SharedPtr data) {
    RCLCPP_DEBUG(get_logger(), "[%s] Recieved a new path with %lu points",
                      name_.c_str(), data->poses.size());

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
