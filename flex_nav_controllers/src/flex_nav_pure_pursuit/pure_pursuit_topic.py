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

class PurePursuitTopic(PurePursuit):
    """
    Pure Pursuit topic listener

    Args:
        name (str): The name of the node
    """

    def __init__(self, name):
        super(PurePursuitTopic, self).__init__(name)

        self._server = actionlib.SimpleActionServer(self._action_name, FollowTopicAction, execute_cb = self.execute, auto_start = False)
        self._server.start()

        self._current_path = None
        self._latest_path = None

        rospy.spin()

    def execute(self, goal):
        """
        The call back for the FollowTopicActionServer

        Args:
            goal (FollowTopicGoal): The goal to process
        """

        super(PurePursuitTopic, self).execute(goal)

        r = rospy.Rate(self._controller_rate)
        result = FollowTopicResult()

        rospy.loginfo('[%s] Attempting to listen to topic: %s', self._name, goal.topic.data)

        good = False

        for topic in rospy.get_published_topics():
            if goal.topic.data in topic[0] and topic[1] == 'nav_msgs/Path':
                self._sub = rospy.Subscriber(goal.topic.data, Path, self.topic_cb)

                rospy.loginfo('[%s] Success!', self._name)
                good = True
                break

        # This is not good
        if not good:
            rospy.logerr('[%s] Desired topic does not publish a nav_msgs/Path', self._name)
            self.set_aborted(result)
            return

        del good

        # Wait for a path to be published
        while not self._current_path and not rospy.is_shutdown() and not self._is_new_goal and not self._server.is_preempt_requested():
            self._current_path = self._latest_path
            r.sleep()

        # This is where the actual work gets done
        self._running = True
        while self._running and not rospy.is_shutdown() and not self._is_new_goal and not self._server.is_preempt_requested():
            self._start_time = rospy.Time.now()
            self._done = False
            self._failed = False

            self._current_path = self._latest_path
            rospy.logdebug('[%s] Executing path with %d points: #%d', self._name, len(self._current_path.poses), self._current_path.header.seq)

            if not self._last_odom_msg:
                rospy.logerr('[%s] No odometry message received', self._name)
                return self.set_aborted(result)

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
                rospy.logerr('[%s] Invalid index %d - cannot access the path points!', self._name, self._indice)
                return self.set_aborted(result)

            while self._current_path.header.stamp == self._latest_path.header.stamp and self._running and not rospy.is_shutdown() and not self._is_new_goal and not self._server.is_preempt_requested():
                # Get latest position and use this throughout calculations
                self._last_odom = self._last_odom_msg
                self._location = self.transformFrame(self._last_odom, self._current_path.header.frame_id)

                if self._failed:
                    self.set_aborted(result)

                if not self.get_current_target(self._current_path.poses):
                    # Check for distance to terminal target within lookahead
                    # Get target given valid index
                    final_target = self._current_path.poses[-1]

                    dr = np.sqrt((final_target.pose.position.x - self._location.point.x)**2 + (final_target.pose.position.y - self._location.point.y)**2 )
                    if dr <= self._lookahead_distance:
                        rospy.logdebug('[%s] Found terminal point - success!', self._name)
                        return self.set_succeeded(result)
                    else:
                        rospy.logerr('[%s] Failed to find point along current path - failed!', self._name)
                        self.set_aborted(result)
                        break

                # Calculate the desired twist based on pure pursuit
                self._twist.twist.linear.x = self._desired_velocity

                try:
                    lookahead = None
                    lookahead = self.calculateLineTwist(self._prior, self._target)

                    if lookahead is None:
                        if self._failed:
                            rospy.logerr('[%s] No lookahead - assume failed!', self._name)
                            return self.set_aborted(result)
                        elif self._done:
                            rospy.logerr('[%s] No lookahead - assume done!', self._name)
                            return self.set_succeeded(result)
                except Exception as e:
                    rospy.logerr('[%s] Invalid point data: %s', self._name, str(e))
                    self._running = False
                    return self.set_aborted(result)

                if math.fabs(self._twist.twist.angular.z) > self._max_rotation_rate:
                    self._twist.twist.linear.x = self._desired_velocity * self._max_rotation_rate / math.fabs(self._twist.twist.angular.z)
                    self._twist.twist.angular.z = math.copysign(self._max_rotation_rate, self._twist.twist.angular.z)

                self._twist.header.stamp = rospy.Time.now()
                self._cmd_pub.publish(self._twist)
                self._marker_pub.publish(self._marker)

                r.sleep()

            r.sleep()

        self.set_preempted(result)

    def topic_cb(self, data):
        """
        The call back for the topic subscriber

        Args:
            data (Path): The Path message to process
        """

        rospy.logdebug('[%s]  Recieved a new path with %d points: #%d', self._name, len(data.poses), data.header.seq)

        self._latest_path = data

    def set_succeeded(self, result):
        """
        Sets a goal as succeeded and cleans everything up

        Args:
            result (FollowTopicResult): The result to be published
        """

        super(PurePursuitTopic, self).set_succeeded(result)

        self._current_path = None
        self._latest_path = None

        if self._sub:
            self._sub.unregister()

    def set_aborted(self, result):
        """
        Sets a goal as aborted and cleans everything up

        Args:
            result (FollowTopicResult): The result to be published
        """

        super(PurePursuitTopic, self).set_aborted(result)

        self._current_path = None
        self._latest_path = None

        if self._sub:
            self._sub.unregister()

    def set_preempted(self, result):
        """
        Sets a goal as preempted and cleans everything up

        Args:
            result (FollowTopicResult): The result to be published
        """

        super(PurePursuitTopic, self).set_preempted(result)

        self._current_path = None
        self._latest_path = None

        if self._sub:
            self._sub.unregister()
