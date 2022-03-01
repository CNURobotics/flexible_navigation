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

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from flex_nav_common.action import *
from nav_msgs.msg import Path
from std_msgs.msg import String

class FollowTopicState(EventState):
    """
    Instructs a controller to listen to the topic of a planner

    -- planner_topic        String    The planner to listen to
    -- controller_topic     String    The controller to command

    <= done         Successfully reached the goal
    <= failed       Failed to generate or follow a plan
    <= canceled    The state was canceled

    """

    def __init__(self, planner_topic, controller_topic):
        """
        Constructor
        """
        super(FollowTopicState, self).__init__(outcomes=['done', 'failed', 'canceled'])

        ProxyActionClient._initialize(FollowTopicState._node)

        self._planner_topic = planner_topic
        self._controller_topic = controller_topic
        self._client = ProxyActionClient({self._controller_topic: FollowTopic})
        self._return = None

    def execute(self, userdata):
        """
        Execute this state
        """

        if self._client.has_result(self._controller_topic):
            result = self._client.get_result(self._controller_topic)

            # Clear so we don't spam if the transition is blocked due to low autonomy level
            ProxyActionClient._result[self._controller_topic] = None

            if result.code == 0:
                Logger.loginfo('%s  Success!' % (self.name))
                self._return = 'done'
            elif result.code == 1:
                Logger.logerr('%s  Failure' % (self.name))
                self._return = 'failed'
            elif result.code == 2:
                Logger.logerr('%s  canceled' % (self.name))
                self._return = 'canceled'
            else:
                Logger.logerr('%s  Unknown error' % (self.name))
                self._return = 'failed'

        return self._return

    def on_enter(self, userdata):
        """
        On enter, send action goal
        """

        self._return = None # Reset the completion flag

        topic = String(data = self._planner_topic)
        result = FollowTopic.Goal(topic = topic)

        try:
            Logger.loginfo('[%s] - Listening to topic: %s' %  (self.name, self._planner_topic))
            self._client.send_goal(self._controller_topic, result)
        except Exception as e:
            Logger.logwarn('[%s] - Failed to listen to topic: %s' % str(e))
            return 'failed'

    def on_exit(self, userdata):
        if self._controller_topic in ProxyActionClient._result:
            ProxyActionClient._result[self._controller_topic] = None

        if self._client.is_active(self._controller_topic):
            Logger.logerr('%s  Canceling active goal' % (self.name))
            self._client.cancel(self._controller_topic)
