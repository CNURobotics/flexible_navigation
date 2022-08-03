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
 *       COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABL*E FOR ANY DIRECT, INDIRECT,
 *       INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *       BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *       LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *       CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *       LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
 *       WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *       POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

/**
 * @class GetPath
 * @brief An extension of base_global_planner that generates a top-level Path
 *
 *   GetPath accepts Poses indicating the desired goal point provided by way
 *   of a GetPathAction. GetPath also has the ability to clear the local
 *   cost map by sending a ClearCostmapAction. Both abilities are handled by
 *   their own ActionServer interfaces and are accessible through the topic
 *   root and `clear_costmap` respectively.
 */

#ifndef FLEX_PLANNER_GET_PATH_H
#define FLEX_PLANNER_GET_PATH_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_navfn_planner/navfn_planner.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.h"
#include "flex_nav_common/action/clear_costmap.hpp"
#include "flex_nav_common/action/get_path.hpp"

#include <pluginlib/class_loader.hpp>

namespace flex_nav {
using ClearCostmapAction = flex_nav_common::action::ClearCostmap;
using ClearCostmapActionServer = nav2_util::SimpleActionServer<ClearCostmapAction>;

using GetPathAction = flex_nav_common::action::GetPath;
using GetPathActionServer = nav2_util::SimpleActionServer<GetPathAction>;


class GetPath : public nav2_util::LifecycleNode {
public:
  /**
   * @brief The constructor to instantiate a node
   * @param tf A reference to a TransformListener
   */
  GetPath(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief The destructor to tear down a node
   */
  ~GetPath();

protected:
  /**
   * @brief Configure member variables and initializes planner
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Activate member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Deactivate member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Reset member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Called when in shutdown state
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  // Planner
  nav2_core::GlobalPlanner::Ptr planner_;
  pluginlib::ClassLoader<nav2_core::GlobalPlanner> gp_loader_;
  std::string default_id_;
  std::string default_type_;
  std::string planner_id_;
  std::string planner_type_;
  double max_planner_duration_;

  // Clock
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  // TF buffer
  std::shared_ptr<tf2_ros::Buffer> tf_;

  // Global Costmap
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::unique_ptr<nav2_util::NodeThread> costmap_thread_;
  nav2_costmap_2d::Costmap2D * costmap_;

  // Publishers for the path
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr plan_publisher_;

private:
  /**
   * @brief The call back for the GetPathActionServer
   * @param goal A reference to the published goal
   */
  void execute();

  /**
   * @brief The call back for the ClearCostmapActionServer
   * @param goal A reference to the published goal
   */
  void clear_costmap();

  std::unique_ptr<GetPathActionServer> gp_server_;
  std::unique_ptr<ClearCostmapActionServer> cc_server_;

  std::string name_;
};
};

#endif
