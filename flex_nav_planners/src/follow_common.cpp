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

#include "flex_nav_planners/follow_common.hpp"

namespace flex_nav
{

int64_t getTargetPointFromPath(
  rclcpp::Logger logger, const double radius_squared_distance,
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const std::vector<geometry_msgs::msg::PoseStamped> & planned_path,
  geometry_msgs::msg::PoseStamped & target_point, const int64_t start_ndx)
{
  double x_diff = 0.0;
  double y_diff = 0.0;
  double sq_dist = 0.0;

  int64_t i = start_ndx;
  while (i < planned_path.size()) {
    sq_dist = distanceSquared(robot_pose, planned_path[i]);

    if (sq_dist <= radius_squared_distance) {
      break;  //  found first point within radius
    }

    ++i;
  }

  if (i == planned_path.size()) {
    // It is possible that localization shift has broken the prior start usage
    // Look backwards for a valid starting point
    i = start_ndx - 1;  // we know start was not valid, so try prior
    while (i >= 0) {
      sq_dist = distanceSquared(robot_pose, planned_path[i]);

      if (sq_dist <= radius_squared_distance) {
        break;  //  found first point within radius
      }

      --i;
    }

    if (i < 0) {
      RCLCPP_ERROR(
        logger,
        "getTargetPointFromPath: Cannot find valid starting point between %ld and end of path at "
        "%ld - invalid target!",
        start_ndx, i);
      return -1;  // no valid target
    }
  }

  while (i < planned_path.size()) {
    sq_dist = distanceSquared(robot_pose, planned_path[i]);

    if (sq_dist > radius_squared_distance) {
      break;  // found the first point outside of the radius
    }

    ++i;
  }

  // i is index of point beyond the target (possibly out of bounds for final point)
  target_point.pose = planned_path[i - 1].pose;

  if (i == planned_path.size()) {
    return i - 1;  // Final pose in path is the last target
  }

  // Planner target is between valid points along the path
  const geometry_msgs::msg::Pose p0 = planned_path[i - 1].pose;
  const geometry_msgs::msg::Pose p1 = planned_path[i].pose;
  geometry_msgs::msg::Point pi;
  double si = intersectCircleLineSegment(
    radius_squared_distance, robot_pose.pose.position, p0.position, p1.position, pi);

  if (si < 0.0) {
    double d0 = std::pow((p0.position.x - robot_pose.pose.position.x), 2) +
                std::pow((p0.position.y - robot_pose.pose.position.y), 2);
    double d1 = std::pow((p1.position.x - robot_pose.pose.position.x), 2) +
                std::pow((p1.position.y - robot_pose.pose.position.y), 2);

    RCLCPP_DEBUG(
      logger,
      "getTargetPointFromPath: Cannot interpolate points at %ld of %ld - from p0=(%.3f, %.3f) "
      "d^2=(%.3f) to p1=(%.3f, %.3f) d2=(%.3f) at (%.3f, %.3f) r2=%.3f - si = %.6f - invalid "
      "target!",
      i, planned_path.size(), p0.position.x, p0.position.y, d0, p1.position.x, p1.position.y, d1,
      robot_pose.pose.position.x, robot_pose.pose.position.y, radius_squared_distance, si);
  } else {
    tf2::Quaternion q0;
    tf2::Quaternion q1;
    tf2::fromMsg(p0.orientation, q0);
    tf2::fromMsg(p1.orientation, q1);
    target_point.pose.position = pi;
    target_point.pose.orientation = tf2::toMsg(q0.slerp(q1, si));
  }

  return i - 1;  // index of valid point within lookahead
}

/**
 * @brief Set orientations based on path segments
 */
void setPathOrientations(std::vector<geometry_msgs::msg::PoseStamped> & poses)
{
  // Set first point orientation based only on next point
  tf2::Vector3 v0, v1, v2;
  tf2::fromMsg(poses[1].pose.position, v1);
  tf2::fromMsg(poses[0].pose.position, v0);
  v0 = v1 - v0;
  calcOrientationFromVector(v0, poses[0].pose.orientation);

  // Set intermediate points based on weighted average of vector into and out of the point
  for (int i = 1; i < poses.size() - 1; i++) {
    // Assume last point has desired goal orientation, but process the others along the path
    // if they are the identity quaternion -
    //    we assume non-identity quaternions are valid orientations
    if (fabs(poses[i].pose.orientation.w - 1.0) < 1.e-6) {
      tf2::fromMsg(poses[i + 1].pose.position, v2);
      tf2::fromMsg(poses[i].pose.position, v1);
      tf2::fromMsg(poses[i - 1].pose.position, v0);

      v0 = v1 - v0;
      v1 = v2 - v1;

      // Scale vector by the square of other length so that resulting "average" orientation
      // is closer to that of the shorter segment,
      // but protect from any 0 length vectors (e.g. pure rotation)
      calcOrientationFromVector(
        (v0 * (v1.length2() + 0.001) + v1 * (v0.length2() + 0.001)), poses[i].pose.orientation);
    }
  }
}

/**
 * @brief transform robot pose into specified frame
 */
bool transformRobot(
  rclcpp::Logger logger, const tf2_ros::Buffer & tf,
  const geometry_msgs::msg::PoseStamped & current_pose,
  geometry_msgs::msg::PoseStamped & transformed_pose, const std::string & frame_id)
{
  try {
    tf.transform(current_pose, transformed_pose, frame_id);
    return true;
  } catch (tf2::LookupException & ex) {
    RCLCPP_ERROR(logger, "No Transform available Error looking up robot pose: %s\n", ex.what());
    return false;
  } catch (tf2::ConnectivityException & ex) {
    RCLCPP_ERROR(logger, "Connectivity Error looking up robot pose: %s\n", ex.what());
    return false;
  } catch (tf2::ExtrapolationException & ex) {
    RCLCPP_ERROR(logger, "Extrapolation Error looking up robot pose: %s\n", ex.what());
    return false;
  }
}

}  // namespace flex_nav
