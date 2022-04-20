#!/usr/bin/env python

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

import rclpy
from rclpy.duration import Duration
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
from flexbe_core.proxy import ProxySubscriberCached

from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class TimedStopState(EventState):
    '''
    This state publishes a constant zero Twist and/or TwistStamped command based on parameters.
    The state monitors the robot odometry message and returns a failed outcome
    if speed is not near zero within the timeout

    -- timeout              float     Time which needs to have passed since the behavior started. (default: 0.25)
    -- cmd_topic            string    topic name of the robot velocity command (default: 'cmd_vel')
    -- odom_topic           string    topic of the robot odometry message (default: 'odom')
    -- cmd_topic_stamped    string    optional topic name of the robot stamped velocity command (default: '')
    <= done         Robot stopped within the specified time.
    <= failed       The robot is still moving according to the odometry message after timeout.
    '''

    def __init__(self, timeout=0.25, cmd_topic='cmd_vel', odom_topic='odom', cmd_topic_stamped=""):
        # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
        super(TimedStopState, self).__init__(outcomes = ['done', 'failed'])

        ProxyPublisher._initialize(TimedStopState._node)
        ProxySubscriberCached._initialize(TimedStopState._node)

        # Store state parameter for later use.
        self._timeout = Duration(seconds=timeout)

        # The constructor is called when building the state machine, not when actually starting the behavior.
        # Thus, we cannot save the starting time now and will do so later.
        self._start_time = None

        self._done = None # Track the outcome so we can detect if transition is blocked

        self._odom_topic = odom_topic
        self._odom_sub = ProxySubscriberCached({self._odom_topic: Odometry})

        if isinstance(cmd_topic, str) and len(cmd_topic) != 0:
            self._twist = Twist() # defaults to zero
            self._cmd_topic = cmd_topic
            self._pub = ProxyPublisher({self._cmd_topic: Twist})
        else:
            self._twist = None
            self._cmd_topic = None
            self._pub = None


        if isinstance(cmd_topic_stamped, str) and len(cmd_topic_stamped) != 0:
            self._twist_stamped  = TwistStamped()
            self._cmd_topic_stamped = cmd_topic_stamped
            self._pub_stamped  = ProxyPublisher({self._cmd_topic_stamped: TwistStamped})
        else:
            self._twist_stamped  = None
            self._cmd_topic_stamped = None
            self._pub_stamped  = None

        if self._pub is None and self._pub_stamped is None:
            Logger.logerr("Must define at least one cmd or cmd_stamped publishing topic")
        if self._cmd_topic == self._cmd_topic_stamped:
            Logger.logerr("Must define differnt names for cmd_topic and cmd_topic_stamped topics")

        assert self._pub or self._pub_stamped, "Must define at least one cmd publishing topic"
        assert self._cmd_topic != self._cmd_topic_stamped, "Must be different topic names!"

    def execute(self, userdata):
        # This method is called periodically while the state is active.
        # If no outcome is returned, the state will stay active.
        if (self._done):
            # We have completed the state, and therefore must be blocked by autonomy level
            # publish commands and return the prior outcome

            if self._pub:
                self._pub.publish(self._cmd_topic, self._twist)

            if self._pub_stamped:
                self._twist_stamped.header.stamp = self._node.get_clock().now().to_msg()  # update the time stamp
                self._pub_stamped.publish(self._cmd_topic_stamped, self._twist_stamped)

            return self._done


        if self._node.get_clock().now().nanoseconds - self._start_time.nanoseconds > self._timeout.nanoseconds:
            # Normal completion, verify stoppage
            if (self._sub.has_msg(self._odom_topic)):
                odom = self._sub.get_last_msg(self._odom_topic)
                speed = odom.twist.twist.linear.x*odom.twist.twist.linear.x + odom.twist.twist.angular.z*odom.twist.twist.angular.z
                if (speed > 5.0e-4):
                    Logger.logwarn('Timed Stop failed - current twist: linear = %f,%f,%f angular=%f, %f, %f' %
                        (odom.twist.twist.linear.x,  odom.twist.twist.linear.y,  odom.twist.twist.linear.z,
                         odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z))
                    self._done = 'failed'
                    return 'failed'
                else:
                    self._done = 'done'
                    return 'done'
            else:
                Logger.logwarn('Timed Stop failed - no odometry feedback!')
                self._done = 'failed'
                return 'failed'


        # Normal operation publish the zero twist
        if self._pub:
            self._pub.publish(self._cmd_topic, self._twist)

        if self._pub_stamped:
            self._twist_stamped.header.stamp = self._node.get_clock().now().to_msg()  # update the time stamp
            self._pub_stamped.publish(self._cmd_topic_stamped, self._twist_stamped)

        return None

    def on_enter(self, userdata):
        # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
        if self._pub:
            self._pub.publish(self._cmd_topic, self._twist)

        if self._pub_stamped:
            self._twist_stamped.header.stamp = self._node.get_clock().now().to_msg()  # update the time stamp
            self._pub_stamped.publish(self._cmd_topic_stamped, self._twist_stamped)

        self._start_time = self._node.get_clock().now()
        self._done       = None # reset the completion flag
