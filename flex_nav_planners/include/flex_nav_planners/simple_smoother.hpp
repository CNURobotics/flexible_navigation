// Copyright (c) 2022, Samsung Research America
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

// This code based on simple_smoother.cpp from
// https://github.com/ros-planning/navigation2.git commit abf3e85d05ac1dbb3a10bfda4afe37cf780184f4

#ifndef FLEX_NAV_PLANNERS__SIMPLE_SMOOTHER_HPP_
#define FLEX_NAV_PLANNERS__SIMPLE_SMOOTHER_HPP_

#include <cmath>
#include <vector>
#include <string>
#include <iostream>
#include <memory>
#include <queue>
#include <utility>

#include "nav2_smoother/smoother_utils.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "angles/angles.h"
#include "tf2/utils.h"

namespace flex_nav
{

/**
 * @class flex_nav_planners::SimpleSmoother
 * @brief A path smoother implementation
 */
class SimpleSmoother
{
public:
  using Ptr = std::shared_ptr<SimpleSmoother>;

  /**
   * @brief A constructor for nav2_smoother::SimpleSmoother
   */
  SimpleSmoother() : max_time_(1, 0)
  {}

  /**
   * @brief A destructor for nav2_smoother::SimpleSmoother
   */
  ~SimpleSmoother() = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &);

  /**
   * @brief Method to cleanup resources.
   */
  void cleanup() {costmap_sub_.reset();}

  /**
   * @brief Method to activate smoother and any threads involved in execution.
   */
  void activate() {}

  /**
   * @brief Method to deactivate smoother and any threads involved in execution.
   */
  void deactivate() {}

  /**
   * @brief Method to smooth given path
   *
   * @param path In-out path to be smoothed
   * @param max_time Maximum duration smoothing should take
   * @return If smoothing was completed (true) or interrupted by time limit (false)
   */
  bool smooth(
    nav_msgs::msg::Path & path,
    const nav2_costmap_2d::Costmap2D * costmap);

protected:
  /**
   * @brief Smoother method - does the smoothing on a segment
   * @param path Reference to path
   * @param reversing_segment Return if this is a reversing segment
   * @param costmap Pointer to minimal costmap
   * @param max_time Maximum time to compute, stop early if over limit
   * @return If smoothing was successful
   */
  bool smoothImpl(
    nav_msgs::msg::Path & path,
    bool & reversing_segment,
    const nav2_costmap_2d::Costmap2D * costmap,
    const double & max_time);

  /**
   * @brief Get the field value for a given dimension
   * @param msg Current pose to sample
   * @param dim Dimension ID of interest
   * @return dim value
   */
  inline double getFieldByDim(
    const geometry_msgs::msg::PoseStamped & msg,
    const unsigned int & dim);

  /**
   * @brief Set the field value for a given dimension
   * @param msg Current pose to sample
   * @param dim Dimension ID of interest
   * @param value to set the dimention to for the pose
   */
  inline void setFieldByDim(
    geometry_msgs::msg::PoseStamped & msg, const unsigned int dim,
    const double & value);

  double tolerance_, data_w_, smooth_w_;
  int max_its_, refinement_ctr_, refinement_num_;
  bool do_refinement_;
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
  rclcpp::Logger logger_{rclcpp::get_logger("SimpleSmoother")};
  rclcpp::Duration max_time_;
};

}  // namespace flex_nav

#endif  // FLEX_NAV_PLANNERS__SIMPLE_SMOOTHER_HPP_
