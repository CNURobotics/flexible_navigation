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

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from flex_nav_common.action import *

class GetPathState(EventState):
    """
    Creates a plan to reach the desired goal point.

    -- planner_topic    String          The planner talk to

    ># goal             PoseStamped     Desired PoseStamped goal

    #> plan             Path            The plan

    <= planned         Successfully created a plan
    <= empty           The generated plan is empty
    <= failed          Failed to create a plan

    """

    def __init__(self, planner_topic):
        """
        Constructor
        """
        super(GetPathState, self).__init__(outcomes=['planned', 'empty', 'failed'], input_keys=['goal'], output_keys=['plan'])

        ProxyActionClient._initialize(GetPathState._node)

        self._action_topic = planner_topic
        self._client = ProxyActionClient({self._action_topic: GetPath})
        self._return = None

    def execute(self, userdata):
        """
        Execute this state
        """

        if self._client.has_result(self._action_topic):
            result = self._client.get_result(self._action_topic)

            # Clear result so we can avoid spam if blocked by autonomy level
            ProxyActionClient._result[self._action_topic] = None

            # Set the appropriate return value
            if result.code == 0:
                Logger.loginfo('%s  Got a plan!' % (self.name))
                userdata.plan = result.plan
                self._return = 'planned'

            elif result.code == 1:
                Logger.logerr('%s    Received an empty plan' % (self.name))
                userdata.plan = None
                self._return = 'empty'

            elif result.code == 2:
                Logger.logerr('%s    Failed to create a plan' % (self.name))
                userdata.plan = None
                self._return = 'failed'

            else:
                Logger.logerr('%s    Unknown error' % (self.name))
                userdata.plan = None
                self._return = 'failed'

        # Return stored value in case we are blocked
        return self._return

    def on_enter(self, userdata):
        """Upon entering the state, send the footstep plan request."""

        self._return = None
        result = GetPath.Goal(pose = userdata.goal)

        try:
            Logger.loginfo('%s    Requesting a plan' % (self.name))
            self._client.send_goal(self._action_topic, result)
        except Exception as e:
            Logger.logwarn('%s    Failed to send plan request: %s' % (self.name, str(e)))
            userdata.plan = None
            return 'failed'

    def on_exit(self, userdata):
        if self._action_topic in ProxyActionClient._result:
            ProxyActionClient._result[self._action_topic] = None

        if self._client.is_active(self._action_topic):
            Logger.logerr('%s    Canceling active goal' % (self.name) )
            self._client.cancel(self._action_topic)
