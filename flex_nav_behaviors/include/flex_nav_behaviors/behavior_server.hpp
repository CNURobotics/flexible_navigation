// Copyright (c) 2022
//    Capable Humanitarian Robotics and Intelligent Systems Lab (CHRISLab)
//     Christopher Newport University
// Copyright (c) 2018 Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License. Reserved.

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "nav2_core/behavior.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"

#ifndef FLEX_NAV_BEHAVIORS__BEHAVIOR_SERVER_HPP_
#define FLEX_NAV_BEHAVIORS__BEHAVIOR_SERVER_HPP_

namespace flex_nav_behavior_server
{

class BehaviorServer : public nav2_util::LifecycleNode
{
public:
  explicit BehaviorServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~BehaviorServer();

  bool loadBehaviorPlugins();

protected:
  // Implement the lifecycle interface
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;

  // Plugins
  std::vector<pluginlib::UniquePtr<nav2_core::Behavior>> behaviors_;
  pluginlib::ClassLoader<nav2_core::Behavior> plugin_loader_;
  std::vector<std::string> default_ids_;
  std::vector<std::string> default_types_;
  std::vector<std::string> behavior_ids_;
  std::vector<std::string> behavior_types_;

  // Utilities
  std::unique_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
  std::unique_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub_;
  std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> collision_checker_;

  double transform_tolerance_;
  std::string name_;
};

}  // namespace flex_nav_behavior_server

#endif  // FLEX_NAV_BEHAVIORS__BEHAVIOR_SERVER_HPP_
