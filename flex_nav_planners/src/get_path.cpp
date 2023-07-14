//  Copyright (c) 2016-2023
//  Capable Humanitarian Robotics and Intelligent Systems Lab (CHRISLab)
//  Christopher Newport University
//
//  All rights reserved.
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions are met:
//
//    1. Redistributions of source code must retain the above copyright notice,
//       this list of conditions and the following disclaimer.
//
//    2. Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//
//    3. Neither the name of the copyright holder nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
//
//       THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//       "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//       LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//       FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//       COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//       INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//       BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//       LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//       CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//       LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
//       WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//       POSSIBILITY OF SUCH DAMAGE.

#include <cstring>
#include <exception>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>

#include "nav2_util/costmap.hpp"
#include "flex_nav_planners/follow_common.hpp"
#include "flex_nav_planners/get_path.hpp"

namespace flex_nav
{
GetPath::GetPath(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("get_path", "", options),
  gp_loader_("nav2_core", "nav2_core::GlobalPlanner"),
  default_id_{"GridBased"},
  default_type_{"nav2_navfn_planner/NavfnPlanner"},
  costmap_(nullptr),
  name_("get_path"),
  set_orientation_(false),
  smoother_type_("")
{
  using namespace std::placeholders;

  declare_parameter("planner_plugin", rclcpp::ParameterValue(default_id_));
  declare_parameter("expected_planner_frequency", rclcpp::ParameterValue(1.0));
  declare_parameter("set_orientation", rclcpp::ParameterValue(false));
  declare_parameter("smoother_type", rclcpp::ParameterValue(smoother_type_));

  costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "global_costmap", std::string{get_namespace()}, "global_costmap");

  costmap_thread_ = std::make_unique<nav2_util::NodeThread>(costmap_ros_);
}

GetPath::~GetPath()
{
  planner_.reset();
  smoother_.reset();
  costmap_thread_.reset();
}

nav2_util::CallbackReturn GetPath::on_configure(const rclcpp_lifecycle::State & state)
{
  name_ = this->get_name();
  RCLCPP_INFO(get_logger(), "Configuring %s ... ", name_.c_str());
  gp_server_ = std::make_unique<GetPathActionServer>(
    get_node_base_interface(), get_node_clock_interface(), get_node_logging_interface(),
    get_node_waitables_interface(), name_, std::bind(&GetPath::execute, this));

  RCLCPP_DEBUG(get_logger(), "Configuring %s cc server ... ", name_.c_str());
  cc_server_ = std::make_unique<ClearCostmapActionServer>(
    get_node_base_interface(), get_node_clock_interface(), get_node_logging_interface(),
    get_node_waitables_interface(), name_ + "/clear_costmap",
    std::bind(&GetPath::clear_costmap, this));

  RCLCPP_DEBUG(get_logger(), "Configuring %s costmap_ros ... ", name_.c_str());

  costmap_ros_->on_configure(state);
  costmap_ = costmap_ros_->getCostmap();

  RCLCPP_INFO(
    get_logger(), "Costmap size: %d,%d", costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());

  tf_ = costmap_ros_->getTfBuffer();

  get_parameter("set_orientation", set_orientation_);
  RCLCPP_INFO(get_logger(), "Set orientations = %d", set_orientation_);

  auto node = shared_from_this();

  // Configure simple smoother if requested
  get_parameter("smoother_type", smoother_type_);
  if (smoother_type_.find("imple") != std::string::npos) {
    // matches simple, Simple, SimpleSmoother, ... (or "pimple")
    RCLCPP_INFO(get_logger(), "smoother type: %s", smoother_type_.c_str());

    try {
      smoother_ = std::make_shared<SimpleSmoother>();
      smoother_->configure(node);
    } catch (const std::exception & ex) {
      RCLCPP_INFO(get_logger(), "Failed to create the simple smoother\n    %s", ex.what());
    } catch (...) {
      RCLCPP_INFO(get_logger(), "Failed to create simple smoother due to unknown error for %s!",
                                smoother_type_.c_str());
    }
  } else if (smoother_type_.find("rientation") != std::string::npos) {
    RCLCPP_INFO(get_logger(), "Use simple orientation calculation instead of simple smoother");
    set_orientation_ = true;
  } else if (smoother_type_ != "" ) {
      RCLCPP_INFO(get_logger(), "Unknown smoother type '%s' - only SimpleSmoother is supported!",
                                smoother_type_.c_str());
      throw std::runtime_error("Uknown smoother type in GetPath ");
  }

  RCLCPP_DEBUG(get_logger(), "Configuring %s planner plugin ... ", name_.c_str());
  get_parameter("planner_plugin", planner_id_);
  if (planner_id_ == default_id_) {
    declare_parameter(default_id_ + ".plugin", rclcpp::ParameterValue(default_type_));
  }

  RCLCPP_INFO(
    get_logger(), "Costmap %s global frame = %s", name_.c_str(),
    costmap_ros_->getGlobalFrameID().c_str());
  try {
    planner_type_ = nav2_util::get_plugin_type_param(node, planner_id_);
    planner_ = gp_loader_.createUniqueInstance(planner_type_);
    RCLCPP_INFO(
      get_logger(), "Created global planner plugin %s of type %s", planner_id_.c_str(),
      planner_type_.c_str());
    planner_->configure(node, planner_id_, tf_, costmap_ros_);
  } catch (const pluginlib::PluginlibException & ex) {
    RCLCPP_FATAL(get_logger(), "Failed to create global planner. Exception: %s", ex.what());
    return nav2_util::CallbackReturn::FAILURE;
  }

  double expected_planner_frequency;
  get_parameter("expected_planner_frequency", expected_planner_frequency);
  if (expected_planner_frequency > 0) {
    max_planner_duration_ = 1 / expected_planner_frequency;
  } else {
    RCLCPP_WARN(
      get_logger(),
      "The expected planner frequency parameter is %.4f Hz. The value should to be greater"
      " than 0.0 to turn on duration overrrun warning messages",
      expected_planner_frequency);
    max_planner_duration_ = 0.0;
  }

  RCLCPP_DEBUG(get_logger(), "Create %s plan publisher ... ", name_.c_str());

  plan_publisher_ = create_publisher<nav_msgs::msg::Path>(name_ + "/plan", 1);

  RCLCPP_DEBUG(get_logger(), "Configure SUCCESS for %s", name_.c_str());
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn GetPath::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Activating %s", name_.c_str());
  plan_publisher_->on_activate();
  gp_server_->activate();
  cc_server_->activate();
  RCLCPP_INFO(get_logger(), "  Activating  cost map for %s", name_.c_str());
  costmap_ros_->on_activate(state);

  RCLCPP_INFO(get_logger(), "   Activating planner for %s", name_.c_str());
  planner_->activate();

  if (smoother_type_ != "") {
    RCLCPP_INFO(get_logger(), "   Activating smoother for %s", name_.c_str());
    smoother_->activate();
  }

  // create bond connection with nav2_util::LifeCycle manager
  createBond();

  RCLCPP_DEBUG(get_logger(), "Activating SUCCESS for %s", name_.c_str());
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn GetPath::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Deactivating %s", name_.c_str());
  plan_publisher_->on_deactivate();
  gp_server_->deactivate();
  cc_server_->deactivate();
  costmap_ros_->on_deactivate(state);

  planner_->deactivate();

  smoother_->deactivate();

  // destroy bond connection with nav2_util::LifeCycle manager
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn GetPath::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Cleaning up %s", name_.c_str());
  plan_publisher_.reset();
  gp_server_.reset();
  cc_server_.reset();
  tf_.reset();
  costmap_ros_->on_cleanup(state);

  smoother_->cleanup();
  planner_->cleanup();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn GetPath::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down %s", name_.c_str());
  return nav2_util::CallbackReturn::SUCCESS;
}

void GetPath::execute()
{
  auto goal = gp_server_->get_current_goal();

  RCLCPP_INFO(get_logger(), "Received goal");
  if (gp_server_ == nullptr || !gp_server_->is_server_active()) {
    RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
    return;
  }

  if (gp_server_->is_cancel_requested()) {
    RCLCPP_INFO(get_logger(), "Goal was canceled. Canceling planning action.");
    gp_server_->terminate_all();
    return;
  }

  // Get current position of the robot
  geometry_msgs::msg::PoseStamped start_pose;
  if (!costmap_ros_->getRobotPose(start_pose)) {
    gp_server_->terminate_current();
    return;
  }

  if (gp_server_->is_cancel_requested()) {
    goal = gp_server_->accept_pending_goal();
  }

  RCLCPP_INFO(
    get_logger(), "Current location (%f, %f)", start_pose.pose.position.x,
    start_pose.pose.position.y);

  RCLCPP_INFO(
    get_logger(), "Generating a path to (%f, %f), qz=%f", goal->pose.pose.position.x,
    goal->pose.pose.position.y, goal->pose.pose.orientation.z);

  auto feedback = std::make_shared<GetPathAction::Feedback>();
  feedback->location = start_pose.pose;
  feedback->goal = goal->pose.pose;
  gp_server_->publish_feedback(feedback);

  std::shared_ptr<flex_nav_common::action::GetPath::Result> result =
    std::make_shared<flex_nav_common::action::GetPath::Result>();
  result->plan.header.stamp = steady_clock_.now();



  // Create a plan from the current robot position to given goal
  nav_msgs::msg::Path path = planner_->createPlan(start_pose, goal->pose);
  if (!path.poses.empty()) {
    if (path.header.frame_id == "") {
      RCLCPP_WARN(get_logger(), "Path frame id is empty");
    }

    // Smooth the plan
     bool smooth_success = false;
    if (smoother_) {
      try {
        smooth_success = smoother_->smooth(path, costmap_ros_->getCostmap());
      } catch (std::runtime_error& e) {
        RCLCPP_WARN(get_logger(), "Exception thrown while smoothing: %s", e.what());
        smooth_success = false;
      }

      if (smooth_success) {
        RCLCPP_DEBUG(get_logger(), "Smoothing success");
      } else {
        RCLCPP_WARN(get_logger(), "Smoothing failed");
      }
    }

    // If using simple 2D planner, set orientations along path based on average change
    if (set_orientation_ && !smooth_success) {
      if (path.poses.size() > 2 && fabs(path.poses[1].pose.orientation.w - 1.0) < 1.e-6) {
        RCLCPP_DEBUG(get_logger(), "Setting orientations along path");
        setPathOrientations(path.poses);
      }
    }
    result->plan = path;
    result->code = flex_nav_common::action::GetPath::Result::SUCCESS;
    plan_publisher_->publish(std::move(path));
    gp_server_->succeeded_current(result);
  } else {
    result->code = flex_nav_common::action::GetPath::Result::FAILURE;
    gp_server_->terminate_current(result);
  }
}

void GetPath::clear_costmap()
{
  auto goal = cc_server_->get_current_goal();
  costmap_ros_->resetLayers();
  std::shared_ptr<flex_nav_common::action::ClearCostmap::Result> result =
    std::make_shared<flex_nav_common::action::ClearCostmap::Result>();
  result->code = flex_nav_common::action::ClearCostmap::Result::SUCCESS;
  cc_server_->succeeded_current(result);
}
}  // namespace flex_nav
