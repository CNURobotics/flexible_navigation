###############################################################################
#  Copyright (c) 2016
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

from flex_nav_common.msg import *
from nav_msgs.msg import Path
from pure_pursuit import PurePursuit
from visualization_msgs.msg import Marker
import actionlib
import math
import numpy as np
import rospy

class PurePursuitPath(PurePursuit):
    """
    Pure Pursuit Path follower

    Args:
        name (str): The name of the node
    """

    def __init__(self, name):
        super(PurePursuitPath, self).__init__(name)

        self._server = actionlib.SimpleActionServer(self._action_name, FollowPathAction, execute_cb = self.execute, auto_start = False)
        self._server.start()

        rospy.spin()

    def execute(self, goal):
        """
        The call back for the FollowPathActionServer

        Args:
            goal (FollowPathGoal): The goal to process
        """

        super(PurePursuitPath, self).execute(goal)

        r = rospy.Rate(self._controller_rate)
        result = FollowPathResult()

        # Wait for the lock
        self._is_new_goal = True
        while self._running and rospy.is_shutdown():
            rospy.logwarn_throttle(0.25, '[%s] Waiting for lock', self._name)
        self._is_new_goal = False

        if not self._last_odom_msg:
            rospy.logerr('[%s] No odometry message received', self._name)
            self.set_aborted(result)
            self._running = False
            return

        self._marker.action = Marker.ADD
        self._marker.color.a = 1.0
        self._marker.color.r = 0.0
        self._marker.color.g = 0.0
        self._marker.color.b = 1.0
        self._marker_pub.publish(self._marker)

        self._start_time = rospy.Time.now()
        self._done = False
        self._failed = False

        self._marker.action = Marker.MODIFY
        self._marker.color.a = 1.0
        self._marker.color.r = 0.0
        self._marker.color.g = 1.0
        self._marker.color.b = 0.0
        self._marker_pub.publish(self._marker)

        self._indice = 1
        if self._indice < len(goal.path.poses):
            self._target.point = goal.path.poses[self._indice].pose.position
            self._prior.point = goal.path.poses[self._indice - 1].pose.position
        else:
            rospy.logerr(
                '[%s] Invalid index %d - cannot access the path points!',
                self._name, self._indice
            )
            self._target = None
            self._prior = None
            self._failed = True
            self._done = True
            self.set_aborted(result)
            self._running = False
            return

        self._running = True
        while self._running and not rospy.is_shutdown() and not self._is_new_goal and not self._server.is_preempt_requested():
            if self._done:
                ts = TwistStamped()
                ts.header.stamp = rospy.Time.now()
                self._cmd_pub.publish(ts)
                self.set_succeeded(result)
                return # Done

            # Current robot location in map frame
            self._location = self.transformMap(self._last_odom_msg)
            if self._failed:
                self.set_aborted(result)
                return # Failed to get transform

            if not self.get_current_target(goal.path.poses):
                # Check for distance to terminal target within lookahead
                # Get target given valid index
                local_target = goal.path.poses[-1]

                dr = np.sqrt((self._target.point.x - self._location.point.x)**2 + (self._target.point.y - self._location.point.y)**2 )
                if dr >= self._lookahead_distance:
                    # Success!
                    rospy.logdebug('[%s] Found terminal point - success!', self._name)
                    self.set_succeeded(result)
                    self._running = False
                    return # Done
                else:
                    # Failure
                    rospy.loginfo('[%s] Failed to find point along current path - failed!', self._name)
                    self.set_aborted(result)
                    self._running = False
                    return # Failed to find target

            # Calculate the desired twist based on pure pursuit
            self._twist.twist.linear.x = self._desired_velocity

            lookahead = None
            lookahead = self.calculateLineTwist(self._prior, self._target)

            if lookahead is None:
                if self._failed:
                    rospy.logerr('[%s] No lookahead - assume failed!', self._name)
                    self.set_aborted(result)
                    self._running = False
                    return # Failed to find target
                elif self._done:
                    rospy.logerr('[%s] No lookahead - assume done!', self._name)
                    return self.set_succeeded(result)
                    self._running = False
                    return # Done

            if math.fabs(self._twist.twist.angular.z) > self._max_rotation_rate:
                self._twist.twist.linear.x = self._desired_velocity * self._max_rotation_rate / math.fabs(self._twist.twist.angular.z)
                self._twist.twist.angular.z = math.copysign(self._max_rotation_rate, self._twist.twist.angular.z)

            self._twist.header.stamp = rospy.Time.now()
            self._cmd_pub.publish(self._twist)
            self._marker_pub.publish(self._marker)

            r.sleep()

        self.set_preempted(result)

    def set_succeeded(self, result):
        """
        Sets a goal as succeeded and cleans everything up

        Args:
            result (FollowPathResult): The result to be published
        """

        super(PurePursuitPath, self).set_succeeded(result)

    def set_aborted(self, result):
        """
        Sets a goal as aborted and cleans everything up

        Args:
            result (FollowPathResult): The result to be published
        """

        super(PurePursuitPath, self).set_aborted(result)

    def set_preempted(self, result):
        """
        Sets a goal as preempted and cleans everything up

        Args:
            result (FollowPathResult): The result to be published
        """

        super(PurePursuitPath, self).set_preempted(result)
