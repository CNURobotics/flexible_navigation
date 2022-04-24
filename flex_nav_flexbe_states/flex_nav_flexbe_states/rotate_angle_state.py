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


from __future__ import division
import math
import rclpy
from rclpy.duration import Duration

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher

from geometry_msgs.msg import Twist, TwistStamped, Point, PointStamped
from nav_msgs.msg import Odometry

from flex_nav_common.action import *

class RotateAngleState(EventState):
    '''
    Rotates the robot through given angle in degrees in the given time defined by userdata
    -- target_time       float     Time which needs to have passed since the behavior started
    -- target_angle      float     Angle we want to rotate (degrees)
    -- odometry_topic    string    Odometry topic (default:   'odom')
    -- cmd_topic         string    Topic name of the robot command (default: '/create_node/cmd_vel')
    -- cmd_topic_stamped string    Optional topic name of the robot stamped velocity command (default: '')
    <= done                        Given time has passed.
    '''
    def __init__(self, target_time, target_angle=360.0, cmd_topic='cmd_vel', odometry_topic='odom', cmd_topic_stamped=''):
        super(RotateAngleState, self).__init__(outcomes = ['done'])

        ProxyPublisher._initialize(RotateAngleState._node)

        self._target_time           = Duration(seconds=target_time)
        self._target_angle          = target_angle*3.141593/180.0
        self._twist                 = Twist()
        self._twist.linear.x  = 0.0
        self._twist.angular.z = (self._target_angle / target_time)

        self._start_time   = None
        self._return       = None # Track the outcome so we can detect if transition is blocked

        self._pub = ProxyPublisher()
        if isinstance(cmd_topic, str) and len(cmd_topic) != 0:
            self._cmd_topic = cmd_topic
            self._pub.createPublisher( cmd_topic, Twist)
        else:
            self._cmd_topic = None


        if isinstance(cmd_topic_stamped, str) and len(cmd_topic_stamped) != 0:
            self._twist_stamped  = TwistStamped()
            self._twist_stamped.twist.linear.x  = 0.0
            self._twist_stamped.twist.angular.z = self._twist.angular.z
            self._cmd_topic_stamped = cmd_topic_stamped
            self._pub.createPublisher( cmd_topic_stamped, TwistStamped)
        else:
            self._twist_stamped  = None
            self._cmd_topic_stamped = None

        if self._cmd_topic is None and self._cmd_topic_stamped is None:
            Logger.logerr("Must define at least one cmd or cmd_stamped publishing topic")
        if self._cmd_topic == self._cmd_topic_stamped:
            Logger.logerr("Must define differnt names for cmd_topic and cmd_topic_stamped topics")

        assert self._cmd_topic or self._cmd_topic_stamped, "Must define at least one cmd publishing topic"
        assert self._cmd_topic != self._cmd_topic_stamped, "Must be different topic names!"


    def execute(self, userdata):

        if (self._return):
            # We have completed the state, and therefore must be blocked by autonomy level
            # Stop the robot, but and return the prior outcome
            if self._cmd_topic:
                self._pub.publish(self._cmd_topic, Twist())

            if self._cmd_topic_stamped:
                ts = TwistStamped() # Zero twist to stop if blocked
                ts.header.stamp = self._node.get_clock().now().to_msg()  # update the time stamp
                self._pub_stamped.publish(self._cmd_topic_stamped, ts)

            return self._return

        #@TODO - modify to track actual angle instead of target time

        if (self._node.get_clock().now() - self._start_time > self._target_time):
            #The robot will twist in place until the desired time has elapsed
            #Once it has, the state will return done
            self._return = 'done'
            return 'done'

        # Normal execution
        if self._cmd_topic:
            self._pub.publish(self._cmd_topic, self._twist)

        if self._cmd_topic_stamped:
            self._twist_stamped.header.stamp = self._node.get_clock().now().to_msg()  # update the time stamp
            self._pub_stamped.publish(self._cmd_topic_stamped, self._twist_stamped)

        return None

    def on_enter(self, userdata):
        self._start_time = self._node.get_clock().now()
        self._return     = None # reset the completion flag
