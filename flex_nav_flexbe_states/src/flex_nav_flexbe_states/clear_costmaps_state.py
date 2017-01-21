#!/usr/bin/env python
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
    -- costmap_topics    string      topic name of the costmap to be cleared. (default: '/flex_nav_planner_node/clear_costmap')
    <= done                          Given time has passed.
    <= failed                        Was not able to clear a desired costmap
    '''

    def __init__(self, costmap_topics =['/high_level_planner/clear_costmap', '/mid_level_planner/clear_costmap', '/low_level_planner/clear_costmap'] ):
        super(ClearCostmapsState, self).__init__(outcomes = ['done', 'failed'])

        self._topics = costmap_topics

        self._clients_list = []
        for topic in self._topics:
            client = ProxyActionClient({topic: ClearCostmapAction})
            self._clients_list.append(client)

    def execute(self, userdata):

        for i in range(len(self._clients_list)):
            try:
                self._clients_list[i].send_goal(self._topics[i], ClearCostmapGoal)
                Logger.loginfo('clearing costmap of topic: %s' %(self._topics[i]) )
            except:
                Logger.logwarn('Was not able to clear costmap of topic: %s' %(self._topics[i]))
                return 'failed'

        return 'done'

    def on_enter(self, userdata):
        #upon entering the state the robot will clear its costmap


        self._start_time = rospy.Time.now()
        self._done       = None # reset the completion flag
