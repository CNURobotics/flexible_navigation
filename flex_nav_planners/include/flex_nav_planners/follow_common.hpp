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

#ifndef FLEX_NAV_PLANNERS__FOLLOW_COMMON_HPP_
#define FLEX_NAV_PLANNERS__FOLLOW_COMMON_HPP_

#include <tf2_ros/buffer.h>

#include <algorithm>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>



namespace flex_nav
{
/**
 * @brief Finds the squared distance between two PoseStamped positions (in the same frame!)
 * @param p1 The first point
 * @param p2 The second point
 * @return The distance between p1 and p2 in double precision
 */
inline double distanceSquared(
  const geometry_msgs::msg::PoseStamped & p1, const geometry_msgs::msg::PoseStamped & p2)
{
  // These points must be in the same frame!
  const double x1 = p1.pose.position.x;
  const double y1 = p1.pose.position.y;
  const double x2 = p2.pose.position.x;
  const double y2 = p2.pose.position.y;
  const double a = x2 - x1;
  const double b = y2 - y1;

  return a * a + b * b;
}

/**
 * @brief Finds intersection of circle with line segment and sets point furtherest along the segment
 * @param radius_squared The lookahead radius squared to search along path
 * @param pc The point of the center of circle
 * @param p0 The starting point of directed segment
 * @param p1 The ending point of the directed segment
 * @return intersection fraction if valid intersection in segment, or negative otherwise
 */
inline double intersectCircleLineSegment(
  const double radius_squared, const geometry_msgs::msg::Point & pc,
  const geometry_msgs::msg::Point & p0, const geometry_msgs::msg::Point & p1,
  geometry_msgs::msg::Point & pi)
{
  const double x0 = p0.x;
  const double y0 = p0.y;
  const double x1 = p1.x;
  const double y1 = p1.y;
  const double xc = pc.x;
  const double yc = pc.y;
  const double dx = x1 - x0;
  const double dy = y1 - y0;

  const double a = (dx * dx) + (dy * dy);
  const double b = (2 * dx * x0) - (2 * dx * xc) + (2 * dy * y0) - (2 * dy * yc);
  const double c =
    -radius_squared + (x0 * x0) - (2 * x0 * xc) + (xc * xc) + (y0 * y0) - (2 * y0 * yc) + (yc * yc);

  const double disc = (b * b) - (4 * a * c);

  if (disc < 0.0) {
    return -1.0;  // no valid intersection in segment
  } else {
    const double s1 = (-b + std::sqrt(disc)) / (2 * a);
    const double s2 = (-b - std::sqrt(disc)) / (2 * a);
    const double smin = std::min(s1, s2);
    const double smax = std::max(s1, s2);

    double sint = smax;  // use smax target if choice
    if (smax < 0.0 || smax > 1.0) {
      sint = smin;  // try smin target instead
    }

    if (sint >= 0.0 && sint <= 1.0) {
      pi.x = (x0 + sint * dx);
      pi.y = (y0 + sint * dy);
      pi.z = (p0.z + sint * (p1.z - p0.z));
      return sint;
    }
  }
  return -1.0;  // no intersection in the segment
}

/**
 * @brief Calculate the orientation of a 2D vector
 *
 */
inline void calcOrientationFromVector(tf2::Vector3 vec, geometry_msgs::msg::Quaternion & quat)
{
  if (vec.length2() > 0.0) {
    vec.normalize();
    double half_yaw = atan2(vec.y(), vec.x()) / 2.0;
    double cha = cos(half_yaw);
    double sha = sin(half_yaw);

    quat.w = cha;
    quat.x = 0.0;
    quat.y = 0.0;
    quat.z = sha;
  }
}

/**
 * @brief Set orientations based on path segments
 * @param poses of the path
 */
void setPathOrientations(std::vector<geometry_msgs::msg::PoseStamped> & poses);

/**
 * @brief Finds a point along path at lookahead radius
 * @param logger The logger for this node
 * @param radius_squared The lookahead radius squared to search along path
 * @param robot_pose The Pose of the robot in the global frame
 * @param planned_path The Path to follow
 * @param target_point The located goal point
 * @param start_ndx  Where to start searching along path, default = 0
 * @return index of target point in path if successful, or negative if invalid
 */
int64_t getTargetPointFromPath(
  rclcpp::Logger logger, const double radius_squared,
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const std::vector<geometry_msgs::msg::PoseStamped> & planned_path,
  geometry_msgs::msg::PoseStamped & target_point, const int64_t start_ndx = 0);

/**
 * @brief transform robot pose into specified frame
 * @param logger The logger for this node
 */
bool transformRobot(
  rclcpp::Logger logger, const tf2_ros::Buffer & tf,
  const geometry_msgs::msg::PoseStamped & current_pose,
  geometry_msgs::msg::PoseStamped & transformed_pose, const std::string & frame_id);
}  // namespace flex_nav
#endif  // FLEX_NAV_PLANNERS__FOLLOW_COMMON_HPP_
