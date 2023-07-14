#!/usr/bin/env python

###############################################################################
#  Copyright (c) 2016-2023
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

from flex_nav_common.action import FollowPath


class FollowPathState(EventState):
    """
    Follow a plan to reach the desired goal point.

    -- topic        String    The node to communicate with

    ># plan         Path      Desired Path to follow

    <= done         Successfully reached the goal
    <= failed       Failed to generate or follow a plan
    <= canceled    The state was canceled

    """

    def __init__(self, topic):
        """Construct state."""
        super().__init__(outcomes=['done', 'failed', 'canceled'], input_keys=['plan'])

        ProxyActionClient.initialize(FollowPathState._node)

        self._action_topic = topic
        self._client = ProxyActionClient({self._action_topic: FollowPath})

    def execute(self, userdata):
        """Execute this state."""
        if self._client.has_result(self._action_topic):
            result = self._client.get_result(self._action_topic)
            ProxyActionClient._result[self._action_topic] = None  # Clear to avoid spam if blocked by low autonomy level
            if result.code == 0:
                Logger.loginfo('%s   Success!' % (self.name))
                self._return = 'done'
            elif result.code == 1:
                Logger.logerr('%s   Failure' % (self.name))
                self._return = 'failed'
            elif result.code == 2:
                Logger.logerr('%s   canceled' % (self.name))
                self._return = 'canceled'
            else:
                Logger.logerr('%s   Unknown error' % (self.name))
                self._return = 'failed'

        # Return prior value if blocked
        return self._return

    def on_enter(self, userdata):
        """On enter, send action goal."""
        self._return = None
        result = FollowPath.Goal(path=userdata.plan)

        try:
            Logger.loginfo('%s   Following the path to victory!' % (self.name))
            self._client.send_goal(self._action_topic, result)
            Logger.loginfo("Sent goal")
        except Exception as e:
            Logger.logwarn('%s   Failed to follow path: %s' % (self.name, str(e)))
            return 'failed'

    def on_exit(self, userdata):
        if self._action_topic in ProxyActionClient._result:
            ProxyActionClient._result[self._action_topic] = None

        if self._client.is_active(self._action_topic):
            Logger.logerr('%s   Canceling active goal' % (self.name))
            self._client.cancel(self._action_topic)

    def on_stop(self):
        """Will be executed once when the behavior stops or is preempted."""
        if self._client.is_active(self._action_topic):
            Logger.logerr('%s   Canceling active goal on SM stop' % (self.name))
            self._client.cancel(self._action_topic)

    def on_pause(self):
        """Will be executed each time this state is paused."""
        if self._client.is_active(self._action_topic):
            Logger.logerr('%s   Canceling active goal on SM pause' % (self.name))
            self._client.cancel(self._action_topic)
