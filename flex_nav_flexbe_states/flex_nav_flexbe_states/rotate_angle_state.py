#!/usr/bin/env python

###############################################################################
#  Copyright (c) 2016-2017
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

from geometry_msgs.msg import TwistStamped, Point, PointStamped
from nav_msgs.msg import Odometry

from flex_nav_common.action import *

class RotateAngleState(EventState):
    '''
    Rotates the robot through given angle in degrees in the given time defined by userdata
    -- target_time      float       Time which needs to have passed since the behavior started
    -- target_angle      float      Angle we want to rotate
    -- odometry_topic   string      topic of the iRobot Create sensor state (default:   '/create_node/odom')
    -- cmd_topic        string      Topic name of the robot command (default: '/create_node/cmd_vel')
    <= done                         Given time has passed.
    '''
    def __init__(self, target_time, target_angle=360.0, cmd_topic='/create_node/cmd_vel', odometry_topic='/create_node/odom'):
        super(RotateAngleState, self).__init__(outcomes = ['done'])

        ProxyPublisher._initialize(RotateAngleState._node)

        self._target_time           = Duration(seconds=target_time)
        self._target_angle          = target_angle*3.141593/180.0
        self._twist                 = TwistStamped()
        self._twist.twist.linear.x  = 0.0
        self._twist.twist.angular.z = (self._target_angle / target_time)

        self._cmd_topic    = cmd_topic
        self._pub          = ProxyPublisher({self._cmd_topic: TwistStamped})
        self._start_time   = None
        self._return       = None # Track the outcome so we can detect if transition is blocked

    def execute(self, userdata):

        if (self._return):
            # We have completed the state, and therefore must be blocked by autonomy level
            # Stop the robot, but and return the prior outcome
            ts = TwistStamped()
            ts.header.stamp = self._node.get_clock().now().to_msg()
            self._pub.publish(self._cmd_topic, ts)
            return self._return

        #@TODO - modify to track actual angle instead of target time

        if (self._node.get_clock().now() - self._start_time > self._target_time):
            #The robot will twist in place until the desired time has elapsed
            #Once it has the it will return done
            self._return = 'done'
            return 'done'

        # Normal execution
        self._twist.header.stamp = self._node.get_clock().now().to_msg()
        self._pub.publish(self._cmd_topic, self._twist)
        return None

    def on_enter(self, userdata):
        self._start_time = self._node.get_clock().now()
        self._return     = None # reset the completion flag
