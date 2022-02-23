#!/usr/bin/env python

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

import rclpy
from rclpy.duration import Duration
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
from flexbe_core.proxy import ProxySubscriberCached
from flexbe_core.proxy import ProxyServiceCaller
from flexbe_core.proxy import ProxyActionClient

from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist


class TimedTwistState(EventState):
    '''
    This state publishes an open loop constant TwistStamped command based on parameters.

    -- target_time     float     Time which needs to have passed since the behavior started.
    -- velocity        float     Body velocity (m/s)
    -- rotation_rate   float     Angular rotation (radians/s)
    -- cmd_topic       string    topic name of the robot velocity command (default: 'cmd_vel')
    <= done                 Given time has passed.
    '''

    def __init__(self, target_time, velocity, rotation_rate, cmd_topic='cmd_vel'):
        # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
        super(TimedTwistState, self).__init__(outcomes = ['done'])

        ProxyPublisher._initialize(TimedTwistState._node)
        ProxySubscriberCached._initialize(TimedTwistState._node)
        ProxyActionClient._initialize(TimedTwistState._node)
        ProxyServiceCaller._initialize(TimedTwistState._node)

        # Store state parameter for later use.
        self._target_time           = Duration(seconds=target_time)
        # self._twist                 = TwistStamped()
        self._twist                 = Twist()

        # self._twist.twist.linear.x  = velocity
        # self._twist.twist.angular.z = rotation_rate

        self._twist.linear.x  = velocity
        self._twist.angular.z = rotation_rate

        # The constructor is called when building the state machine, not when actually starting the behavior.
        # Thus, we cannot save the starting time now and will do so later.
        self._start_time = None

        self._done       = None # Track the outcome so we can detect if transition is blocked

        self._cmd_topic    = cmd_topic
        self._pub          = ProxyPublisher({self._cmd_topic: Twist})

    def execute(self, userdata):
        # This method is called periodically while the state is active.
        # If no outcome is returned, the state will stay active.

        if (self._done):
            # We have completed the state, and therefore must be blocked by autonomy level
            # Stop the robot, but and return the prior outcome
            ts = TwistStamped() # Zero twist to stop if blocked
            ts.header.stamp = self._node.get_clock().now().to_msg()

            self._pub.publish(self._cmd_topic, ts)
            return self._done

        if self._node.get_clock().now().nanoseconds - self._start_time.nanoseconds > self._target_time.nanoseconds:
            # Normal completion, do not bother repeating the publish
            self._done = 'done'
            return 'done'

        # Normal operation
        # self._twist.header.stamp = self._node.get_clock().now().to_msg()  # update the timestamp
        self._pub.publish(self._cmd_topic, self._twist)
        return None

    def on_enter(self, userdata):
        # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
        self._start_time = self._node.get_clock().now()
        self._done       = None # reset the completion flag
