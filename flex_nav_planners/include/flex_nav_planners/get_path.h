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

#include <ros/ros.h>
#include <tf2_ros/buffer.h>

#include <actionlib/server/simple_action_server.h>
#include <flex_nav_common/ClearCostmapAction.h>
#include <flex_nav_common/GetPathAction.h>

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_layer.h>
#include <nav_core/base_global_planner.h>

#include <pluginlib/class_loader.hpp>

namespace flex_nav {
typedef actionlib::SimpleActionServer<flex_nav_common::ClearCostmapAction>
    ClearCostmapActionServer;
typedef actionlib::SimpleActionServer<flex_nav_common::GetPathAction>
    GetPathActionServer;

class GetPath {
public:
  /**
   * @brief The constructor to instantiate a node
   * @param tf A reference to a TransformListener
   */
  GetPath(tf2_ros::Buffer& tf);

  /**
   * @brief The destructor to tear down a node
   */
  ~GetPath();

private:
  /**
   * @brief The call back for the GetPathActionServer
   * @param goal A reference to the published goal
   */
  void execute(const flex_nav_common::GetPathGoalConstPtr &goal);

  /**
   * @brief The call back for the ClearCostmapActionServer
   * @param goal A reference to the published goal
   */
  void clear_costmap(const flex_nav_common::ClearCostmapGoalConstPtr &goal);

  tf2_ros::Buffer &tf_;
  GetPathActionServer *gp_server_;
  ClearCostmapActionServer *cc_server_;
  costmap_2d::Costmap2DROS *costmap_;
  pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> loader_;
  boost::shared_ptr<nav_core::BaseGlobalPlanner> planner_;

  std::string robot_base_frame_, global_frame_, name_;
  double planner_frequency_, inscribed_radius_, circumscribed_radius_;
  double planner_patience_, oscillation_timeout_, oscillation_distance_;
};
};

#endif
