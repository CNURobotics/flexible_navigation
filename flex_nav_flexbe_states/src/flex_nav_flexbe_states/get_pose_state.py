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

import rospy
from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxySubscriberCached

from geometry_msgs.msg import PoseStamped

class GetPoseState(EventState):
    """
    Grabs the most recent published PoseStamped.

    -- topic    String           The topic to subscribe to

    #> goal     PointStamped     The goal.

    <= done     Goal PoseStamped is available.

    """

    def __init__(self, topic = 'move_base_simple/goal'):
        """Constructor"""
        super(GetPoseState, self).__init__(outcomes=['done'], output_keys=['goal'])

        self._topic = topic
        self._sub = ProxySubscriberCached({self._topic: PoseStamped})
        self._goal_pose = None

    def execute(self, userdata):
        if self._sub.has_msg(self._topic):
            Logger.loginfo('%s  Received new goal' % (self.name))
            self._goal_pose = self._sub.get_last_msg(self._topic)
            self._sub.remove_last_msg(self._topic)

        if self._goal_pose is not None:
            userdata.goal = self._goal_pose
            return 'done'

    def on_enter(self, userdata):
        userdata.goal = None
        self._goal_pose = None
        if self._sub.has_msg(self._topic):
            Logger.loginfo('%s  Clearing prior goal' % (self.name))
            self._sub.remove_last_msg(self._topic)
        Logger.loginfo('%s Input new goal ' % (self.name))
