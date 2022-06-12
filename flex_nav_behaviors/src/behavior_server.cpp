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

#include <memory>
#include <string>
#include <vector>
#include <utility>
#include "nav2_util/node_utils.hpp"
#include "flex_nav_behaviors/behavior_server.hpp"

namespace flex_nav_behavior_server
{

BehaviorServer::BehaviorServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("behaviors_server", "", options),
  plugin_loader_("nav2_core", "nav2_core::Behavior"),
  default_ids_{"spin", "backup", "drive_on_heading", "wait"},
  default_types_{"nav2_behaviors/Spin",
                 "nav2_behaviors/BackUp",
                 "nav2_behaviors/DriveOnHeading",
                 "nav2_behaviors/Wait"}
{
  declare_parameter(
    "costmap_topic",
    rclcpp::ParameterValue(std::string("local_costmap/costmap_raw")));
  declare_parameter(
    "footprint_topic",
    rclcpp::ParameterValue(std::string("local_costmap/published_footprint")));
  declare_parameter("cycle_frequency", rclcpp::ParameterValue(10.0));
  declare_parameter("behavior_plugins", default_ids_);

  get_parameter("behavior_plugins", behavior_ids_);
  if (behavior_ids_ == default_ids_) {
    for (size_t i = 0; i < default_ids_.size(); ++i) {
      declare_parameter(default_ids_[i] + ".plugin", default_types_[i]);
    }
  }

  declare_parameter(
    "global_frame",
    rclcpp::ParameterValue(std::string("odom")));
  declare_parameter(
    "robot_base_frame",
    rclcpp::ParameterValue(std::string("base_link")));
  declare_parameter(
    "transform_tolerance",
    rclcpp::ParameterValue(0.1));
}


BehaviorServer::~BehaviorServer()
{
  behaviors_.clear();
}

nav2_util::CallbackReturn
BehaviorServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  name_ = this->get_name();
  RCLCPP_INFO(get_logger(), "Configuring %s", name_.c_str());

  tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(),
    get_node_timers_interface());
  tf_->setCreateTimerInterface(timer_interface);
  transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);

  std::string costmap_topic, footprint_topic;
  std::string global_frame, robot_base_frame;
  this->get_parameter("costmap_topic", costmap_topic);
  this->get_parameter("footprint_topic", footprint_topic);
  this->get_parameter("transform_tolerance", transform_tolerance_);
  this->get_parameter("global_frame", global_frame);
  this->get_parameter("robot_base_frame", robot_base_frame);
  costmap_sub_ = std::make_unique<nav2_costmap_2d::CostmapSubscriber>(
    shared_from_this(), costmap_topic);
  footprint_sub_ = std::make_unique<nav2_costmap_2d::FootprintSubscriber>(
    shared_from_this(), footprint_topic, *tf_, robot_base_frame, transform_tolerance_);

  //collision_checker_ = std::make_shared<nav2_costmap_2d::CostmapTopicCollisionChecker>(
  //  *costmap_sub_, *footprint_sub_, *tf_, this->get_name());
  //  //,    global_frame, robot_base_frame, transform_tolerance_);
  collision_checker_ = std::make_shared<nav2_costmap_2d::CostmapTopicCollisionChecker>(
    *costmap_sub_, *footprint_sub_, this->get_name());

  behavior_types_.resize(behavior_ids_.size());
  if (!loadBehaviorPlugins()) {
    return nav2_util::CallbackReturn::FAILURE;
  }

  return nav2_util::CallbackReturn::SUCCESS;
}


bool
BehaviorServer::loadBehaviorPlugins()
{
  auto node = shared_from_this();

  for (size_t i = 0; i != behavior_ids_.size(); i++) {
    behavior_types_[i] = nav2_util::get_plugin_type_param(node, behavior_ids_[i]);
    try {
      RCLCPP_INFO(
        get_logger(), "Creating behavior plugin %s of type %s",
        behavior_ids_[i].c_str(), behavior_types_[i].c_str());
      behaviors_.push_back(plugin_loader_.createUniqueInstance(behavior_types_[i]));
      behaviors_.back()->configure(node, behavior_ids_[i], tf_, collision_checker_);
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(
        get_logger(), "Failed to create behavior %s of type %s."
        " Exception: %s", behavior_ids_[i].c_str(), behavior_types_[i].c_str(),
        ex.what());
      return false;
    }
  }

  return true;
}

nav2_util::CallbackReturn
BehaviorServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating %s", name_.c_str());
  std::vector<pluginlib::UniquePtr<nav2_core::Behavior>>::iterator iter;
  for (iter = behaviors_.begin(); iter != behaviors_.end(); ++iter) {
    (*iter)->activate();
  }

  // create bond connection with nav2_util::LifeCycle manager
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
BehaviorServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating %s", name_.c_str());

  std::vector<pluginlib::UniquePtr<nav2_core::Behavior>>::iterator iter;
  for (iter = behaviors_.begin(); iter != behaviors_.end(); ++iter) {
    (*iter)->deactivate();
  }

  // destroy bond connection with nav2_util::LifeCycle manager
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
BehaviorServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up %s", name_.c_str());

  std::vector<pluginlib::UniquePtr<nav2_core::Behavior>>::iterator iter;
  for (iter = behaviors_.begin(); iter != behaviors_.end(); ++iter) {
    (*iter)->cleanup();
  }

  behaviors_.clear();
  transform_listener_.reset();
  tf_.reset();
  footprint_sub_.reset();
  costmap_sub_.reset();
  collision_checker_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
BehaviorServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down %s", name_.c_str());
  return nav2_util::CallbackReturn::SUCCESS;
}

}  // end namespace flex_nav_behavior_server

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(flex_nav_behavior_server::BehaviorServer)
