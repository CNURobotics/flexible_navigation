/*******************************************************************************
 *  Copyright (c) 2016-2019
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

#include <flex_nav_planners/follow_common.h>

namespace flex_nav {

double distanceSquared(const geometry_msgs::msg::PoseStamped &p1,
                       const geometry_msgs::msg::PoseStamped &p2) {
  const double x1 = p1.pose.position.x;
  const double y1 = p1.pose.position.y;
  const double x2 = p2.pose.position.x;
  const double y2 = p2.pose.position.y;
  const double a = x2 - x1;
  const double b = y2 - y1;

  return a * a + b * b;
}

bool getTargetPointFromPath(
    const double radius, const geometry_msgs::msg::PoseStamped &robot_pose,
    const std::vector<geometry_msgs::msg::PoseStamped> &planned_path,
    geometry_msgs::msg::PoseStamped &target_point) {
  double x_diff = 0.0;
  double y_diff = 0.0;
  double sq_dist = 0.0;

  long i = 0;
  while (i < planned_path.size()) {
    x_diff = robot_pose.pose.position.x -
             planned_path[i].pose.position.x; // map frame calculations
    y_diff = robot_pose.pose.position.y - planned_path[i].pose.position.y;
    sq_dist = x_diff * x_diff + y_diff * y_diff;

    if (sq_dist <= radius) {
      break;
    }

    ++i;
  }

  while (i < planned_path.size()) {
    x_diff = robot_pose.pose.position.x -
             planned_path[i].pose.position.x; // map frame calculations
    y_diff = robot_pose.pose.position.y - planned_path[i].pose.position.y;
    sq_dist = x_diff * x_diff + y_diff * y_diff;

    if (sq_dist > radius) {
      break;
    }

    ++i;
  }

  target_point.pose.position.x = planned_path[i - 1].pose.position.x;
  target_point.pose.position.y = planned_path[i - 1].pose.position.y;
  target_point.pose.position.z = planned_path[i - 1].pose.position.z;
  target_point.pose.orientation = geometry_msgs::msg::Quaternion(); // 0.0, 0.0, 0.0, 1.0);

  if (i == planned_path.size()) {
    return true;
  }

  const geometry_msgs::msg::PoseStamped p0 = planned_path[i - 1];
  const geometry_msgs::msg::PoseStamped p1 = planned_path[i];

  const double x0 = p0.pose.position.x;
  const double y0 = p0.pose.position.y;
  const double x1 = p1.pose.position.x;
  const double y1 = p1.pose.position.y;
  const double xc = robot_pose.pose.position.x;
  const double yc = robot_pose.pose.position.y;
  const double dx = x1 - x0;
  const double dy = y1 - y0;

  const double a = (dx * dx) + (dy * dy);
  const double b =
      (2 * dx * x0) - (2 * dx * xc) + (2 * dy * y0) - (2 * dy * yc);
  const double c = -radius + (x0 * x0) - (2 * x0 * xc) + (xc * xc) + (y0 * y0) -
                   (2 * y0 * yc) + (yc * yc);

  const double disc = (b * b) - (4 * a * c);

  if (disc < 0.0) {
    return false;
  } else {
    const double s1 = (-b + std::sqrt(disc)) / (2 * a);
    const double s2 = (-b - std::sqrt(disc)) / (2 * a);
    const double smax = std::max(s1, s2);
    const double smin = std::min(s1, s2);

    if (smax >= 0.0 && smax <= 1.0) {
      target_point.pose.position.x = (x0 + smax * dx); // map frame
      target_point.pose.position.y = (y0 + smax * dy);
    } else if (smin >= 0.0 && smin <= 1.0) {
      target_point.pose.position.x = (x0 + smin * dx);
      target_point.pose.position.y = (y0 + smin * dy);
    }
  }

  return true;
}

/**
 * @brief transform robot pose into specified frame
 */
bool transformRobot(const tf2_ros::Buffer &tf,
                    const geometry_msgs::msg::PoseStamped &current_pose,
                    geometry_msgs::msg::PoseStamped &transformed_pose,
                    const std::string& frame_id)
{
    try
    {
      tf.transform(current_pose, transformed_pose, frame_id);
      return true;
    }
    catch (tf2::LookupException& ex)
    {
      RCLCPP_ERROR(rclcpp::get_logger("follow_common_logger"), "No Transform available Error looking up robot pose: %s\n", ex.what());
      return false;
    }
    catch (tf2::ConnectivityException& ex)
    {
      RCLCPP_ERROR(rclcpp::get_logger("follow_common_logger"), "Connectivity Error looking up robot pose: %s\n", ex.what());
      return false;
    }
    catch (tf2::ExtrapolationException& ex)
    {
      RCLCPP_ERROR(rclcpp::get_logger("follow_common_logger"), "Extrapolation Error looking up robot pose: %s\n", ex.what());
      return false;
    }

}

}
