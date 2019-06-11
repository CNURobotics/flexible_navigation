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
 *       COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *       INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *       BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *       LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *       CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *       LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
 *       WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *       POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#ifndef FLEX_PLANNER_FOLLOW_COMMON_H
#define FLEX_PLANNER_FOLLOW_COMMON_H

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace flex_nav {
/**
 * @brief Finds the squared distance between two PoseStamped positions
 * @param p1 The first point
 * @param p2 The second point
 * @return The distance between p1 and p2 in double precision
 */
double distanceSquared(const geometry_msgs::PoseStamped &p1,
                       const geometry_msgs::PoseStamped &p2);

/**
 * @brief Finds a point along path at lookahead radius
 * @param radius The radius to search
 * @param robot_pose The Pose of the robot in the global frame
 * @param planned_path The Path to follow
 * @param target_point The located goal point
 * @return True if a target point was successfully found
 */
bool getTargetPointFromPath(
    const double radius, const geometry_msgs::PoseStamped &robot_pose,
    const std::vector<geometry_msgs::PoseStamped> &planned_path,
    geometry_msgs::PoseStamped &target_point);


/**
 * @brief transform robot pose into specified frame
 */
bool transformRobot(const geometry_msgs::PoseStamped &current_pose, const geometry_msgs::PoseStamped &transformed_pose, const string& frame_id);
}
#endif
