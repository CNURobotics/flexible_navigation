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

import importlib
import rosidl_runtime_py.set_message

from flexbe_core import EventState, Logger
from geometry_msgs.msg import TwistStamped

from flexbe_core.proxy import ProxyActionClient

class RecoveryState(EventState):

    '''
    Sends an action goal to a recovery action server
    -- module_name       string      the name of package where the recovery action is defined
    -- class_name        string      the name of the action
    -- goal_data         dictionary  Data to create populate the action goal's fields
    -- topic             string      the topic of action server
    -- timeout           int         time in seconds to wait for a response from the action server
    <= done                          Given time has passed.
    <= failed                        Timeout waiting for a response from the action server
    '''

    def __init__(self, module_name, class_name, goal_data, topic, timeout):
        super(RecoveryState, self).__init__(outcomes = ['done', 'failed'])

        module = importlib.import_module(module_name)
        self.action_class = getattr(module, class_name)
        self.goal = self.action_class.Goal()

        # Populate goal fields with user supplied dictionary
        rosidl_runtime_py.set_message.set_message_fields(self.goal, goal_data)

        self._topic = topic
        self._timeout = timeout
        self._return  = None
        ProxyActionClient._initialize(RecoveryState._node)

        self._client = ProxyActionClient({self._topic: self.action_class})

    def execute(self, userdata):
        elapsed = self._node.get_clock().now() - self._start_time

        if self._client.has_result(self._topic) and elapsed.nanoseconds * 1e-9 > self._timeout:
            self._return = 'done'

        if self._return is None and elapsed.nanoseconds * 1e-9 > self._timeout:
            Logger.logwarn('Timeout waiting to receive result from recovery action server' )
            self._return = 'failed'
            return 'failed'

        # Waiting on action results
        return self._return

    def on_enter(self, userdata):
        #upon entering the state the robot will send the action goal
        self._return  = None

        try:
            self._client.send_goal(self._topic, self.goal)
        except Exception as e:
            Logger.logwarn("Unable to send recovery goal with topic %s" % (self._topic))
            self._return = 'failed'

        self._start_time = self._node.get_clock().now()
