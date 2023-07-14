#!/usr/bin/env python

###############################################################################
#  Copyright (c) 2022-2023
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

from flex_nav_common.action import GetPathByName


class GetPathByNameState(EventState):
    """
    Retrieve a named plan from an action server.

    The state will use an optional path_name on user data if available,
    but this is not required.

    -- action_server_name   String      The paths server talk to
    -- path_name            String      The path name requested

    #> plan             Path            The plan

    <= success         Successfully retrieved a plan
    <= empty           The retrieved plan is empty
    <= failed          Failed to retrieve a plan

    """

    def __init__(self, action_server_name='get_path_by_name', path_name='empty'):
        super().__init__(outcomes=['success', 'empty', 'failed'], output_keys=['plan'])

        ProxyActionClient.initialize(GetPathByNameState._node)

        self._action_server_name = action_server_name
        self._client = ProxyActionClient({self._action_server_name: GetPathByName})
        self._path_name = path_name
        self._return = None

    def execute(self, userdata):
        if self._client.has_result(self._action_server_name):
            result = self._client.get_result(self._action_server_name)

            # Clear result so we can avoid spam if blocked by autonomy level
            ProxyActionClient._result[self._action_server_name] = None

            path_length = len(result.plan.poses)
            # Set the appropriate return value
            if result.code == 0 and path_length > 0:
                Logger.loginfo('%s  Got a plan!' % (self.name))
                userdata.plan = result.plan
                self._return = 'success'

            elif result.code == 1 or path_length == 0:
                Logger.logerr('%s    Received an empty plan' % (self.name))
                userdata.plan = None
                self._return = 'empty'

            elif result.code == 2:
                Logger.logerr('%s    Failed to retrieve a plan' % (self.name))
                userdata.plan = None
                self._return = 'failed'

            else:
                Logger.logerr('%s    Unknown error' % (self.name))
                userdata.plan = None
                self._return = 'failed'

        # Return stored value in case we are blocked
        return self._return

    def on_enter(self, userdata):
        self._return = None
        path_name = self._path_name
        if path_name is None:
            try:
                # Get name from user data if it exists, otherwise
                path_name = userdata.path_name
            except Exception:  # pylint: disable=W0703
                pass

        if path_name is None:
            userdata.plan = None
            return 'failed'

        try:
            Logger.loginfo(f'{self.name}  Requesting the {path_name} ')
            path_goal = GetPathByName.Goal(path_name=path_name)

            self._client.send_goal(self._action_server_name, path_goal)
        except Exception as exc:
            Logger.logwarn('%s    Failed to send plan request: %s' % (self.name, str(exc)))
            userdata.plan = None
            return 'failed'

    def on_exit(self, userdata):
        if self._action_server_name in ProxyActionClient._result:
            ProxyActionClient._result[self._action_server_name] = None

        if self._client.is_active(self._action_server_name):
            Logger.logerr('%s    Canceling active goal' % (self.name))
            self._client.cancel(self._action_server_name)
