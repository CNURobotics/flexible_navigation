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

#include <math.h>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "flex_nav_planners/follow_common.hpp"
#include "flex_nav_planners/follow_path.hpp"

namespace flex_nav
{
FollowPath::FollowPath(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("follow_path", "", options),
  fp_loader_("nav2_core", "nav2_core::GlobalPlanner"),
  default_id_{"GridBased"},
  default_type_{"nav2_navfn_planner/NavfnPlanner"},
  costmap_(nullptr),
  name_("follow_path"),
  running_(false),
  smoother_type_(""),
  set_orientation_(false)
{
  using namespace std::placeholders;

  declare_parameter("planner_plugin", rclcpp::ParameterValue(default_id_));
  declare_parameter("expected_planner_frequency", rclcpp::ParameterValue(1.0));
  declare_parameter("lookahead_distance", rclcpp::ParameterValue(5.0));
  declare_parameter("distance_threshold", rclcpp::ParameterValue(0.5));
  declare_parameter("set_orientation", rclcpp::ParameterValue(false));
  declare_parameter("smoother_type", rclcpp::ParameterValue(smoother_type_));

  declare_parameter("costmap_name", rclcpp::ParameterValue("middle_costmap"));
  declare_parameter("global_frame", rclcpp::ParameterValue("map"));

  get_parameter("costmap_name", costmap_name_);
  get_parameter("global_frame", global_frame_);

  costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    costmap_name_, std::string{get_namespace()}, costmap_name_);

  costmap_thread_ = std::make_unique<nav2_util::NodeThread>(costmap_ros_);
}

FollowPath::~FollowPath()
{
  planner_.reset();
  costmap_thread_.reset();
  smoother_.reset();
}

nav2_util::CallbackReturn FollowPath::on_configure(const rclcpp_lifecycle::State & state)
{
  name_ = this->get_name();
  RCLCPP_INFO(get_logger(), "Configuring %s", name_.c_str());
  fp_server_ = std::make_unique<FollowPathActionServer>(
    get_node_base_interface(), get_node_clock_interface(), get_node_logging_interface(),
    get_node_waitables_interface(), name_, std::bind(&FollowPath::execute, this));

  cc_server_ = std::make_unique<ClearCostmapActionServer>(
    get_node_base_interface(), get_node_clock_interface(), get_node_logging_interface(),
    get_node_waitables_interface(), name_ + "/clear_costmap",
    std::bind(&FollowPath::clear_costmap, this));

  costmap_ros_->on_configure(state);
  costmap_ = costmap_ros_->getCostmap();

  RCLCPP_DEBUG(
    get_logger(), "Costmap size: %d,%d", costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());

  get_parameter("expected_planner_frequency", expected_planner_frequency_);

  // These parameters are now distance in meters (not related to costmap size)
  get_parameter("lookahead_distance", lookahead_distance_squared_);
  get_parameter("distance_threshold", distance_threshold_squared_);

  lookahead_distance_squared_ *= lookahead_distance_squared_;  // as the name implies!
  distance_threshold_squared_ *= distance_threshold_squared_;  // as the name implies!

  tf_ = costmap_ros_->getTfBuffer();

  get_parameter("planner_plugin", planner_id_);
  if (planner_id_ == default_id_) {
    declare_parameter(default_id_ + ".plugin", default_type_);
  }


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

  try {
    planner_type_ = nav2_util::get_plugin_type_param(node, planner_id_);
    planner_ = fp_loader_.createUniqueInstance(planner_type_);
    RCLCPP_INFO(
      get_logger(), "Created global planner plugin %s of type %s", planner_id_.c_str(),
      planner_type_.c_str());
    planner_->configure(node, planner_id_, tf_, costmap_ros_);
  } catch (const pluginlib::PluginlibException & ex) {
    RCLCPP_FATAL(get_logger(), "Failed to create global planner. Exception: %s", ex.what());
    return nav2_util::CallbackReturn::FAILURE;
  }

  if (expected_planner_frequency_ > 0) {
    max_planner_duration_ = 1 / expected_planner_frequency_;
  } else {
    RCLCPP_WARN(
      get_logger(),
      "The expected planner frequency parameter is %.4f Hz. The value should to be greater"
      " than 0.0 to turn on duration overrrun warning messages",
      expected_planner_frequency_);
    max_planner_duration_ = 0.0;
  }

  plan_publisher_ = create_publisher<nav_msgs::msg::Path>(name_ + "/plan", 1);

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn FollowPath::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Activating %s", name_.c_str());
  plan_publisher_->on_activate();
  fp_server_->activate();
  cc_server_->activate();
  costmap_ros_->on_activate(state);
  planner_->activate();

  if (smoother_type_ != "") {
    RCLCPP_INFO(get_logger(), "   Activating smoother for %s", name_.c_str());
    smoother_->activate();
  }

  // create bond connection with nav2_util::LifeCycle manager
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn FollowPath::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Deactivating %s", name_.c_str());
  plan_publisher_->on_deactivate();
  fp_server_->deactivate();
  cc_server_->deactivate();
  costmap_ros_->on_deactivate(state);
  planner_->deactivate();
  smoother_->deactivate();

  // destroy bond connection with nav2_util::LifeCycle manager
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn FollowPath::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Cleaning up %s", name_.c_str());
  plan_publisher_.reset();
  fp_server_.reset();
  cc_server_.reset();
  tf_.reset();
  costmap_ros_->on_cleanup(state);
  planner_->cleanup();
  costmap_ = nullptr;
  smoother_->cleanup();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn FollowPath::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down %s", name_.c_str());
  return nav2_util::CallbackReturn::SUCCESS;
}

void FollowPath::execute()
{
  auto goal = fp_server_->get_current_goal();
  rclcpp::Rate r(expected_planner_frequency_);
  std::shared_ptr<flex_nav_common::action::FollowPath::Result> result =
    std::make_shared<flex_nav_common::action::FollowPath::Result>();

  RCLCPP_INFO(get_logger(), "Received goal");
  if (fp_server_ == nullptr || !fp_server_->is_server_active()) {
    RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
    return;
  }

  if (fp_server_->is_cancel_requested()) {
    RCLCPP_INFO(get_logger(), "Goal was canceled. Canceling planning action.");
    fp_server_->terminate_all();
    return;
  }

  while (running_) {
    RCLCPP_WARN(get_logger(), "Waiting for lock");
    r.sleep();
  }

  if (goal->path.poses.empty()) {
    RCLCPP_ERROR(get_logger(), "The path is empty! - terminate");
    terminateCurrentFollower();
    return;
  }

  if (goal->path.header.frame_id == "") {
    RCLCPP_ERROR(get_logger(), "The frame_id is empty - terminate!");
    terminateCurrentFollower();
    return;
  }

  RCLCPP_INFO(
    get_logger(), "Ready to process latest goal - Cancel requested(%d)",
    fp_server_->is_cancel_requested());

  running_ = true;

  // Get the current robot pose from costmap
  geometry_msgs::msg::PoseStamped current_pose_map;

  if (!costmap_ros_->getRobotPose(current_pose_map)) {
    RCLCPP_ERROR(get_logger(), "No valid current pose to start - terminate!");
    terminateCurrentFollower();
    return;
  }

  // Convert the current pose from map to path frame
  geometry_msgs::msg::PoseStamped current_pose_path;
  if (goal->path.header.frame_id != current_pose_map.header.frame_id) {
    // transform current frame to the path frame to get current target
    if (!transformRobot(
          get_logger(), *tf_, current_pose_map, current_pose_path, goal->path.header.frame_id)) {
      RCLCPP_ERROR(get_logger(), "No valid transform from current pose to path frame - terminate!");

      terminateCurrentFollower();
      return;
    }
  } else {
    current_pose_path = current_pose_map;
  }

  // Find the starting point along the given path
  geometry_msgs::msg::PoseStamped goal_pose_path;

  int64_t path_ndx = getTargetPointFromPath(
    get_logger(), lookahead_distance_squared_, current_pose_path, goal->path.poses, goal_pose_path,
    0);
  if (path_ndx < 0) {
    RCLCPP_ERROR(
      get_logger(), "No valid target along path from the current robot pose - terminate!");

    terminateCurrentFollower();
    return;
  }

  while (running_ && rclcpp::ok() && !fp_server_->is_cancel_requested()) {
    // Get the current robot pose from costmap
    if (!costmap_ros_->getRobotPose(current_pose_map)) {
      terminateCurrentFollower();
      return;
    }

    // Convert the current pose from map to path frame
    if (goal->path.header.frame_id != current_pose_map.header.frame_id) {
      // transform current frame to the path frame to get current target
      if (!transformRobot(
            get_logger(), *tf_, current_pose_map, current_pose_path, goal->path.header.frame_id)) {
        RCLCPP_ERROR(
          get_logger(), "No valid transform from current pose to path frame - terminate!");

        terminateCurrentFollower();
        return;
      }
    } else {
      current_pose_path = current_pose_map;
    }

    // Check to see if we have reached the end of the path
    if (
      distanceSquared(current_pose_path, goal->path.poses.back()) <= distance_threshold_squared_) {
      RCLCPP_INFO(get_logger(), "Reached goal - Success!");
      result->code = flex_nav_common::action::FollowPath::Result::SUCCESS;
      fp_server_->succeeded_current(result);
      running_ = false;

      // Clear path on success
      nav_msgs::msg::Path path;
      path.header.stamp = steady_clock_.now();
      path.header.frame_id = goal->path.header.frame_id;
      plan_publisher_->publish(std::move(path));

      return;
    }

    path_ndx = getTargetPointFromPath(
      get_logger(), lookahead_distance_squared_, current_pose_path, goal->path.poses,
      goal_pose_path, path_ndx);  // start from prior index, forces progress along path
    if (path_ndx < 0) {
      RCLCPP_ERROR(
        get_logger(), "No valid target along path from the current robot pose - terminate!");

      terminateCurrentFollower();
      return;
    }

    // frame ids are not always set along path, and need to update the time stamp as
    // we move along so that transforms are available
    goal_pose_path.header.stamp = steady_clock_.now();
    goal_pose_path.header.frame_id = goal->path.header.frame_id;

    geometry_msgs::msg::PoseStamped goal_pose_map = goal_pose_path;
    // Convert the goal point from path frame to global frame
    if (goal->path.header.frame_id != global_frame_) {
      // goal_pose_map gets the transformed goal pose
      if (!transformRobot(get_logger(), *tf_, goal_pose_path, goal_pose_map, global_frame_)) {
        RCLCPP_ERROR(
          get_logger(),
          "No valid transform for the target point to planner frame found along path - terminate!");

        terminateCurrentFollower();
        return;
      }
    }

    // Update the status of robot while following the path
    auto feedback = std::make_shared<FollowPathAction::Feedback>();
    feedback->pose = goal_pose_map;
    fp_server_->publish_feedback(feedback);

    nav_msgs::msg::Path path = planner_->createPlan(current_pose_map, goal_pose_map);
    if (!path.poses.empty()) {
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

      plan_publisher_->publish(std::move(path));
    } else {
      RCLCPP_WARN(get_logger(), "Empty path - terminate goal!");
      terminateCurrentFollower();
      return;
    }

    r.sleep();  // until next plan update
  }

  if (fp_server_->is_cancel_requested()) {
    RCLCPP_WARN(get_logger(), "Cancel requested - preempting goal ...");

    result->code = flex_nav_common::action::FollowPath::Result::CANCEL;
    fp_server_->terminate_current(result);
  } else {
    RCLCPP_ERROR(get_logger(), "Terminating for unknown reason!");
    terminateCurrentFollower();
  }

  running_ = false;
}

void FollowPath::terminateCurrentFollower()
{
  std::shared_ptr<flex_nav_common::action::FollowPath::Result> result =
    std::make_shared<flex_nav_common::action::FollowPath::Result>();

  result->code = flex_nav_common::action::FollowPath::Result::FAILURE;
  fp_server_->terminate_current(result);
  running_ = false;

  // Clear path on success
  nav_msgs::msg::Path path;
  path.header.stamp = steady_clock_.now();
  plan_publisher_->publish(std::move(path));
}

void FollowPath::clear_costmap()
{
  auto goal = cc_server_->get_current_goal();
  costmap_ros_->resetLayers();
  std::shared_ptr<flex_nav_common::action::ClearCostmap::Result> result =
    std::make_shared<flex_nav_common::action::ClearCostmap::Result>();
  result->code = flex_nav_common::action::ClearCostmap::Result::SUCCESS;
  cc_server_->succeeded_current(result);
}
}  // namespace flex_nav
