#!/usr/bin/env python

###############################################################################
#  Copyright (c) 2016-2022
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

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
from flexbe_core.proxy import ProxySubscriberCached

from geometry_msgs.msg import Twist, TwistStamped, Point, PointStamped
from nav_msgs.msg import Odometry

from flex_nav_common.action import *


class MoveDistanceState(EventState):
    '''
    Implements a state that moves the robot based on userdata
    -- target_time      float       Time which needs to have passed since the behavior started
    -- distance         float       The distance which the robot travels
    -- odometry_topic   string      topic of the odometry message (default:   '/odom')
    -- cmd_topic        string      Topic name of the robot command (default: '/cmd_vel')
    <= done                         Given time has passed.
    <= failed                       The robot was not able to move in the desired amount of time
    '''

    def __init__(self, target_time, distance, cmd_topic='/cmd_vel', odometry_topic='/odom',
                 cmd_topic_stamped=''):
        super(MoveDistanceState, self).__init__(outcomes = ['done', 'failed'])
        self._target_time           = target_time
        self._distance              = distance
        self._velocity              = (distance / target_time)
        self._twist                 = Twist()
        self._twist.linear.x  = self._velocity
        self._twist.angular.z = 0.0

        # The constructor is called when building the state machine, not when actually starting the behavior.
        # Thus, we cannot save the starting time now and will do so later.
        self._start_time = None

        self._return     = None # Track the outcome so we can detect if transition is blocked

        ProxyPublisher._initialize(MoveDistanceState._node)
        ProxySubscriberCached._initialize(MoveDistanceState._node)

        self._odom_topic   = odometry_topic
        self._odom_sub     = ProxySubscriberCached({self._odom_topic:  Odometry})

        self._starting_odom = None

        if isinstance(cmd_topic, str) and len(cmd_topic) != 0:
            self._cmd_topic = cmd_topic
            self._pub = ProxyPublisher({self._cmd_topic: Twist})
        else:
            self._cmd_topic = None
            self._pub = None


        if isinstance(cmd_topic_stamped, str) and len(cmd_topic_stamped) != 0:
            self._twist_stamped  = TwistStamped()
            self._twist_stamped.twist = self._twist
            self._cmd_topic_stamped = cmd_topic_stamped
            self._pub_stamped  = ProxyPublisher({self._cmd_topic_stamped: TwistStamped})
        else:
            self._twist_stamped  = None
            self._cmd_topic_stamped = None
            self._pub_stamped  = None

        if self._pub is None and self._pub_stamped is None:
            Logger.logerr("Must define at least one cmd or cmd_stamped publishing topic")
        if self._cmd_topic == self._cmd_topic_stamped:
            Logger.logerr("Must define differnt names for cmd_topic and cmd_topic_stamped topics")

        assert self._pub or self._pub_stamped, "Must define at least one cmd publishing topic"
        assert self._cmd_topic != self._cmd_topic_stamped, "Must be different topic names!"


    def execute(self, userdata):

        if (self._return):
            # We have completed the state, and therefore must be blocked by autonomy level
            # Stop the robot, but and return the prior outcome
            ts = TwistStamped()
            ts.header.stamp = self._node.get_clock().now().to_msg()
            self._pub.publish(self._cmd_topic, ts)
            return self._return


        #This checks the odom to get the current position,
        # if the robot has moved enough then the state will return
        if (self._sub.has_msg(self._odom_topic)):
            last_odom_msg = self._sub.get_last_msg(self._odom_topic)
            last_posn = last_odom_msg.pose.pose.position
            starting_odom_position = self._starting_odom.pose.pose.position

            # Track how much time has elapsed according to odometry message (float seconds)
            elapsed_time = (last_odom_msg.header.stamp - self._start_time).nanoseconds * 1e-9

            distance_traveled = math.sqrt( (starting_odom_position.x - last_posn.x)**2.0 + (starting_odom_position.y - last_posn.y)**2.0)

            if (self._distance <= 0):
                # Distance traveled is always positive, so use negative multiplier for target check
                if (distance_traveled >= -0.98*self._distance):
                    Logger.loginfo('completed backing up a distance of %f (target %f)' %(-distance_traveled,self._distance))
                    self._return = 'done'
                    return 'done'

                Logger.loginfo('distance traveled %f of %f meters in %f of %f seconds ' %(-distance_traveled,self._distance, elapsed_time, self._target_time))
            else:
                if (distance_traveled >= 0.98*self._distance):
                    Logger.loginfo('completed distance traveled %f (target %f)' %(distance_traveled,self._distance))
                    self._return = 'done'
                    return 'done'

                Logger.loginfo('distance traveled %f of %f meters in %f of %f seconds ' %(distance_traveled,self._distance, elapsed_time, self._target_time))

            if (elapsed_time  > 1.05*self._target_time ):
                # If the time it takes to back up is longer than the desired time
                # then we will return the failure state
                # Slightly extending the target time to allow for message delays
                Logger.logwarn('  timeout before traveling desired distance ' )
                self._return = 'failed'
                return 'failed'

        if (elapsed_time  > 2.0*self._target_time ):
            # If the time it takes to back up is significantly longer than the desired time
            # constraints then will return the failure state
            # Extending the target time to allow for message delays
            Logger.logwarn(' long timeout before traveling desired distance ' )
            self._return = 'failed'
            return 'failed'

        # Normal operation
        self._twist.header.stamp = self._node.get_clock().now().to_msg()
        self._pub.publish(self._cmd_topic, self._twist)
        return None

    def on_enter(self, userdata):
        # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
        if (self._sub.has_msg(self._odom_topic)):
            self._starting_odom = self._sub.get_last_msg(self._odom_topic)
            self._start_time = self._node.get_clock().now()
            self._return       = None # reset the completion flag
        else:
            Logger.logerr(' Cannot move - no odometry message to track!' )
            self._return = 'failed'
