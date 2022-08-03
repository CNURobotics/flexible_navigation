###############################################################################
#  Copyright (c) 2016-2022
#  Capable Humanitarian Robotics and Intelligent Systems Lab (CHRISLab)
#  Christopher Newport University
#
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#
#    1. Redistributions of source code must retain the above copyright notice,
#       this list of conditions and the following disclaimer.
#
#    2. Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#
#    3. Neither the name of the copyright holder nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
#       THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#       "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#       LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#       FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#       COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#       INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#       BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#       LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#       CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#       LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
#       WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#       POSSIBILITY OF SUCH DAMAGE.
###############################################################################

import math
import numpy as np
import rclpy
from rclpy.action import ActionServer
from flex_nav_common.action import *
from nav_msgs.msg import Path
from flex_nav_pure_pursuit.pure_pursuit import PurePursuit
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped

class PurePursuitPath(PurePursuit):
    """
    Pure Pursuit Path follower
    """

    def __init__(self):
        super(PurePursuitPath, self).__init__("pure_pursuit_path")
        self._action_server = ActionServer(self, FollowPath, self._action_name, self.execute)
        # rclpy.spin(self)

    def execute(self, goal_handle):
        """
        The call back for the FollowPathActionServer

        Args:
            goal (FollowPathGoal): The goal to process
        """
        goal = goal_handle.request()
        super(PurePursuitPath, self).execute(goal)
        r = rclpy.Rate(self._controller_rate.value)
        result = FollowPath.Result()

        # Wait for the lock
        self._is_new_goal = True

        while self._running and not rclpy.ok():
            self.get_logger().warn(0.25, '{} Waiting for lock'.format(self.get_name()))

        self._is_new_goal = False

        if not self._last_odom_msg:
            self.get_logger().error('{} No odometry message received'.format(self.get_name()))
            self.set_aborted(goal, result)
            self._running = False
            return

        if self._marker_pub:
            self._marker.action = Marker.ADD
            self._marker.color.a = 1.0
            self._marker.color.r = 0.0
            self._marker.color.g = 0.0
            self._marker.color.b = 1.0
            self._marker_pub.publish(self._marker)

        self._start_time = self.get_clock().now()
        self._done = False
        self._failed = False

        if self._marker_pub:
            self._marker.action = Marker.MODIFY
            self._marker.color.a = 1.0
            self._marker.color.r = 0.0
            self._marker.color.g = 1.0
            self._marker.color.b = 0.0
            self._marker_pub.publish(self._marker)

        self._indice = 1
        if self._indice < len(goal.goal_info.path.poses):
            self._target.point = goal.goal_info.path.poses[self._indice].pose.position
            self._prior.point = goal.goal_info.path.poses[self._indice - 1].pose.position
        else:
            self.get_logger().error(
                '%s Invalid index %d - cannot access the path points!'
                % (self.get_name(), self._indice)
            )
            self._target = None
            self._prior = None
            self._failed = True
            self._done = True
            self.set_aborted(goal, result)
            self._running = False
            return

        self._running = True
        # while self._running and not rospy.is_shutdown() and not self._is_new_goal and not self._server.is_preempt_requested():
        while self._running and rclpy.ok() and not self._is_new_goal and not goal.is_cancel_requested:
            if self._done:
                ts = TwistStamped()
                ts.header.stamp = self.get_clock().now()
                self._cmd_pub.publish(ts)
                self.set_succeeded(goal, result)
                return # Done

            # Current robot location in map frame
            self._location = self.transformFrame(self._last_odom_msg, goal.goal_info.path.header.frame_id)
            if self._failed:
                self.set_aborted(goal, result)
                return # Failed to get transform

            if not self.get_current_target(goal.goal_info.path.poses):
                # Check for distance to terminal target within lookahead
                # Get target given valid index
                local_target = goal.goal_info.path.poses[-1]

                dr = np.sqrt((self._target.point.x - self._location.point.x)**2 + (self._target.point.y - self._location.point.y)**2 )
                if dr >= self._lookahead_distance.value:
                    # Success!
                    self.get_logger().debug('{} Found terminal point - success!'.format(self.get_name()))
                    self.set_succeeded(goal, result)
                    self._running = False
                    return # Done
                else:
                    # Failure
                    self.get_logger().info('{} Failed to find point along current path - failed!'.format(self.get_name()))
                    self.set_aborted(goal, result)
                    self._running = False
                    return # Failed to find target

            # Calculate the desired twist based on pure pursuit
            self._twist.twist.linear.x = self._desired_velocity.value

            lookahead = None
            lookahead = self.calculateLineTwist(self._prior, self._target)

            if lookahead is None:
                if self._failed:
                    self.get_logger().error('{} No lookahead - assume failed!'.format(selfget._name()))
                    self.set_aborted(goal, result)
                    self._running = False
                    return # Failed to find target
                elif self._done:
                    self.get_logger().errpr('{} No lookahead - assume done!'.format(self.get_name()))
                    return self.set_succeeded(goal, result)
                    self._running = False
                    return # Done

            if math.fabs(self._twist.twist.angular.z) > self._max_rotation_rate.value:
                self._twist.twist.linear.x = self._desired_velocity.value * self._max_rotation_rate.value / math.fabs(self._twist.twist.angular.z)
                self._twist.twist.angular.z = math.copysign(self._max_rotation_rate.value, self._twist.twist.angular.z)

            if self._cmd_pub:
                self._cmd_pub.publish(self._cmd_topic, self._twist)

            if self._cmd_pub_stamped:
                self._twist_stamped.header.stamp = rospy.Time.now()
                self._cmd_pub_stamped.publish(self._cmd_topic_stamped, self._twist_stamped)

            if self._marker_pub:
                self._marker_pub.publish(self._marker)

            r.sleep()

        goal.canceled()
        self.set_preempted(result)

    def set_succeeded(self, goal, result):
        """
        Sets a goal as succeeded and cleans everything up

        Args:
            goal (FollowPathGoal): The goal to succeed
            result (FollowPathResult): The result to be published
        """

        goal.succeed()
        super(PurePursuitPath, self).set_succeeded(result)

    def set_aborted(self, goal, result):
        """
        Sets a goal as aborted and cleans everything up

        Args:
            goal (FollowPathGoal): The goal to abort
            result (FollowPathResult): The result to be published
        """

        goal.abort()
        super(PurePursuitPath, self).set_aborted(result)

    def set_preempted(self, goal, result):
        """
        Sets a goal as preempted and cleans everything up

        Args:
            goal (FollowPathGoal): The goal to preempt
            result (FollowPathResult): The result to be published
        """

        goal.canceled()
        super(PurePursuitPath, self).set_preempted(result)

def main(args=None):
	rclpy.init(args=args)
	pure_pursuit_path_node = PurePursuitPath()
	rclpy.spin(pure_pursuit_path_node)
	rclpy.shutdown()

if __name__ == '__main__':
	main()
