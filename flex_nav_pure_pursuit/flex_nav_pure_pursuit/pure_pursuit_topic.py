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

class PurePursuitTopic(PurePursuit):
    """
    Pure Pursuit topic listener
    """

    def __init__(self):
        super(PurePursuitTopic, self).__init__("pure_pursuit_topic")
        self._action_server = ActionServer(self, FollowTopic, self._action_name, self.execute)
        self._current_path = None
        self._latest_path = None

        # rospy.spin()

    def execute(self, goal_handle):
        """
        The call back for the FollowTopicActionServer

        Args:
            goal (FollowTopicGoal): The goal to process
        """

        goal = goal_handle.request()
        super(PurePursuitTopic, self).execute(goal)

        r = rclpy.Rate(self._controller_rate.value)
        result = FollowTopic.Result()
        self.get_logger().info('%s Attempting to listen to topic: %s' % (self._name, goal.topic.data))

        try:
            self._sub = rclpy.create_subscription(Path, goal.topic.data, self.topic_cb, 10)
            self.get_logger().info('{} Success!'.format(self._name))
        except:
            self.get_logger().error('{} Desired topic does not publish a nav_msgs/Path'.format(self._name))
            self.set_aborted(goal, result)
            return

        # Wait for a path to be published
        while not self._current_path and rclpy.ok() and not self._is_new_goal and not goal.is_cancel_requested():
            self._current_path = self._latest_path
            r.sleep()

        # This is where the actual work gets done
        self._running = True
        while self._running and rclpy.ok() and not self._is_new_goal and not goal.is_cancel_requested():
            self._start_time = self.get_clock().now().to_msg()
            self._done = False
            self._failed = False

            self._current_path = self._latest_path
            self.get_logger().debug('%s Executing path with %d points: %d' % (self._name, len(self._current_path.poses), self._current_path.header.seq))

            if not self._last_odom_msg:
                self.get_logger().error('{} No odometry message received'.format(self._name))
                return self.set_aborted(goal, result)

            if self._marker_pub:
                self._marker.action = Marker.MODIFY
                self._marker.color.a = 1.0
                self._marker.color.r = 0.0
                self._marker.color.g = 1.0
                self._marker.color.b = 0.0
                self._marker_pub.publish(self._marker)

            self._indice = 1
            if self._indice < len(self._current_path.poses):
                try:
                    self._target.point = self._current_path.poses[self._indice].pose.position
                    self._target.header = self._current_path.poses[self._indice].header
                    self._prior.point = self._current_path.poses[self._indice - 1].pose.position
                    self._prior.header = self._current_path.poses[self._indice - 1].header
                except:
                    break
            else:
                self.get_logger().error('%s Invalid index %d - cannot access the path points!' % (self._name, self._indice))
                return self.set_aborted(goal, result)

            while self._current_path.header.stamp == self._latest_path.header.stamp and self._running and rclpy.ok() and not self._is_new_goal and not self._server.is_cancel_requested():
                # Get latest position and use this throughout calculations
                self._last_odom = self._last_odom_msg
                self._location = self.transformFrame(self._last_odom, self._current_path.header.frame_id)

                if self._failed:
                    self.set_aborted(goal, result)

                if not self.get_current_target(self._current_path.poses):
                    # Check for distance to terminal target within lookahead
                    # Get target given valid index
                    final_target = self._current_path.poses[-1]

                    dr = np.sqrt((final_target.pose.position.x - self._location.point.x)**2 + (final_target.pose.position.y - self._location.point.y)**2 )
                    if dr <= self._lookahead_distance.value:
                        self.get_logger().debug('{} Found terminal point - success!'.format(self._name))
                        return self.set_succeeded(goal, result)
                    else:
                        self.get_logger().error('{} Failed to find point along current path - failed!'.format(self._name))
                        self.set_aborted(goal, result)
                        break

                # Calculate the desired twist based on pure pursuit
                self._twist.twist.linear.x = self._desired_velocity.value

                try:
                    lookahead = None
                    lookahead = self.calculateLineTwist(self._prior, self._target)

                    if lookahead is None:
                        if self._failed:
                            self.get_logger().error('{} No lookahead - assume failed!'.format(self._name))
                            return self.set_aborted(goal, result)
                        elif self._done:
                            self.get_logger().error('{} No lookahead - assume done!'.format(self._name))
                            return self.set_succeeded(goal, result)
                except Exception as e:
                    self.get_logger().error('%s Invalid point data: %s' % (self._name, str(e)))
                    self._running = False
                    return self.set_aborted(goal, result)

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

            r.sleep()

        self.set_preempted(goal, result)

    def topic_cb(self, data):
        """
        The call back for the topic subscriber

        Args:
            data (Path): The Path message to process
        """

        rospy.logdebug('%s  Received a new path with %d points: %d' % (self._name, len(data.poses), data.header.seq))
        self._latest_path = data

    def set_succeeded(self, goal, result):
        """
        Sets a goal as succeeded and cleans everything up

        Args:
            goal (FolowTopicGoal): The goal to succeed
            result (FollowTopicResult): The result to be published
        """

        goal.succeed()
        super(PurePursuitTopic, self).set_succeeded(result)

        self._current_path = None
        self._latest_path = None

        if self._sub:
            self._sub.destroy()

    def set_aborted(self, goal, result):
        """
        Sets a goal as aborted and cleans everything up

        Args:
            goal (FolowTopicGoal): The goal to abort
            result (FollowTopicResult): The result to be published
        """

        goal.abort()
        super(PurePursuitTopic, self).set_aborted(result)

        self._current_path = None
        self._latest_path = None

        if self._sub:
            self._sub.destroy()

    def set_preempted(self, goal, result):
        """
        Sets a goal as preempted and cleans everything up

        Args:
            goal (FolowTopicGoal): The goal to preempt
            result (FollowTopicResult): The result to be published
        """

        goal.canceled()
        super(PurePursuitTopic, self).set_preempted(result)

        self._current_path = None
        self._latest_path = None

        if self._sub:
            self._sub.destroy()

def main(args=None):
	rclpy.init(args=args)
	pure_pursuit_topic_node = PurePursuitTopic()
	rclpy.spin(pure_pursuit_topic_node)
	rclpy.shutdown()


if __name__ == '__main__':
	main()
