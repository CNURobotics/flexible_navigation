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

import actionlib
import rospy

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller, ProxyActionClient
from geometry_msgs.msg import TwistStamped

from flexbe_core.proxy import ProxyPublisher
from flexbe_core.proxy import ProxySubscriberCached
from flexbe_core.proxy import ProxyServiceCaller
from flexbe_core.proxy import ProxyActionClient

from flex_nav_common.msg import *

class ClearCostmapsState(EventState):

    '''
    Clears the costmaps defined by the userdata
    -- costmap_topics    string      topic name of the costmap to be cleared. (default: ['high_level_planner/clear_costmap', 'low_level_planner/clear_costmap'])
    -- timeout           double      seconds to wait before declaring failure (default: 5.0)
    <= done                          Given time has passed.
    <= failed                        Was not able to clear all costmaps
    '''

    def __init__(self, costmap_topics =['high_level_planner/clear_costmap', 'low_level_planner/clear_costmap'], timeout=5.0 ):
        super(ClearCostmapsState, self).__init__(outcomes = ['done', 'failed'])

        self._topics = costmap_topics
        self._timeout = timeout
        self._return  = None
        self._clients_list = []
        self._costmaps_cleared = []
        for topic in self._topics:
            client = ProxyActionClient({topic: ClearCostmapAction})
            self._clients_list.append(client)
            self._costmaps_cleared.append(False)

    def execute(self, userdata):

        all_costmaps_cleared = True
        for i in range(len(self._clients_list)):
            try:
                if self._clients_list[i].has_result(self._topics[i]):
                    result = self._clients_list[i].get_result(self._topics[i])
                    ProxyActionClient._result[self._topics[i]] = None # Clear to avoid spam if blocked by low autonomy level

                    if result.code == 0:
                        Logger.loginfo('   Successfully cleared %s!'% (self._topics[i]))
                        self._costmaps_cleared[i] = True # We cleared this one
                    elif result.code == 1:
                        Logger.logerr('   Failure to clear %s'% (self._topics[i]))
                    elif result.code == 2:
                        Logger.logerr('   Preempted clearing of %s'% (self._topics[i]))
                    else:
                        Logger.logerr('   Unknown error clearing %s'% (self._topics[i]))
            except:
                Logger.logwarn(' Exception: Was not able to clear costmap of topic  %s ' %(self._topics[i]))

            if (not self._costmaps_cleared[i]):
                # This cost map has not confirmed that it is cleared yet
                all_costmaps_cleared = False

        if (all_costmaps_cleared):
            self._return = 'done'
            return 'done'

        if (self._return is None):
            elapsed = rospy.Time.now() - self._start_time
            if (elapsed.to_sec() > self._timeout):
                Logger.logwarn('Timeout waiting to clear costmaps' )
                self._return = 'failed'
                return 'failed'

        # Waiting on action results
        return self._return

    def on_enter(self, userdata):
        #upon entering the state the robot will clear its costmap
        self._return  = None
        for i in range(len(self._clients_list)):
            try:
                self._costmaps_cleared[i] = False
                self._clients_list[i].send_goal(self._topics[i], ClearCostmapGoal)
                Logger.loginfo('clearing costmap of topic  %s ' %(self._topics[i]) )
            except:
                Logger.logwarn('Was not able to clear costmap of topic  %s ' %(self._topics[i]))

        self._start_time = rospy.Time.now()
