#!/usr/bin/env python

###############################################################################
#  Copyright (c) 2022
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

from flexbe_core import EventState, Logger

from geometry_msgs.msg import PoseStamped

class SetPoseState(EventState):
    """
    Sets a goal pose in userdata

    -- position   float[]    position data ([x, y] or [x, y, z])
    -- angle      float      optional rotation about z-axis angle (radians)
    -- quaternion float[]    quaternion [x, y, z, w]
    -- frame_id   string     frame id (default='map')

    #> goal     PoseStamped     The goal.

    <= done     Goal PoseStamped is available.

    """

    def __init__(self, position, angle=None, quaternion=None, frame_id='map'):
        """Constructor"""
        super(SetPoseState, self).__init__(outcomes=['done'], output_keys=['goal'])

        self._goal_pose = PoseStamped()
        self._goal_pose.header.frame_id = frame_id
        self._goal_pose.pose.position.x = float(position[0]) # x is required
        self._goal_pose.pose.position.y = float(position[1]) # y is required
        try:
            self._goal_pose.pose.position.z = float(position[2]) # y is required
        except:
            pass

        # orientation is optional
        if angle is not None:
            angle = float(angle)
            self._goal_pose.pose.orientation.x = 0.
            self._goal_pose.pose.orientation.y = 0.
            self._goal_pose.pose.orientation.z = math.sin(angle/2.0)
            self._goal_pose.pose.orientation.w = math.cos(angle/2.0)
        elif quaternion is not None:
            # Use of quaternion requires 4 floating point values
            self._goal_pose.pose.orientation.x = float(quaternion[0]) # x is required
            self._goal_pose.pose.orientation.y = float(quaternion[1]) # y is required
            self._goal_pose.pose.orientation.z = float(quaternion[2]) # z is required
            self._goal_pose.pose.orientation.w = float(quaternion[3]) # w is required

    def execute(self, userdata):

        if self._goal_pose is not None:
            self._goal_pose.header.stamp = self._node.get_clock().now().to_msg()
            userdata.goal = self._goal_pose
            return 'done'

    def on_enter(self, userdata):
        Logger.loghint(f'{self.name}  Set new Nav goal @ ({self._goal_pose.pose.position})' )
