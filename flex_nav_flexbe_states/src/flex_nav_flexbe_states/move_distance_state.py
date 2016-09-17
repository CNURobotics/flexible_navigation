#!/usr/bin/env python
from __future__ import division
import rospy
import math

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
from flexbe_core.proxy import ProxySubscriberCached
from flexbe_core.proxy import ProxyServiceCaller
from flexbe_core.proxy import ProxyActionClient

from geometry_msgs.msg import TwistStamped, Point, PointStamped
from nav_msgs.msg import Odometry

from flex_nav_common.msg import *


class MoveDistanceState(EventState):
    '''
    Implements a state that moves the robot based on userdata
    -- target_time      float       Time which needs to have passed since the behavior starte
    -- distance         float       The distance which the robot travels
    -- odometry_topic   string      topic of the iRobot Create sensor state (default:   '/create_node/odom')
    -- cmd_topic        string      Topic name of the robot command (default: '/create_node/cmd_vel')
    <= done                         Given time has passed.
    <= failed                       The robot was not able to move in the desired amount of time
    '''

    def __init__(self, target_time, distance, cmd_topic='/create_node/cmd_vel', odometry_topic='/create_node/odom'):
        super(MoveDistanceState, self).__init__(outcomes = ['done', 'failed'])
        self._target_time           = rospy.Duration(target_time)
        self._distance              = distance
        self._velocity              = (distance / target_time)
        self._twist                 = TwistStamped()
        self._twist.twist.linear.x  = self._velocity
        self._twist.twist.angular.z = 0

        # The constructor is called when building the state machine, not when actually starting the behavior.
        # Thus, we cannot save the starting time now and will do so later.
        self._start_time = None

        self._done       = None # Track the outcome so we can detect if transition is blocked


        self._cmd_topic    = cmd_topic
        self._pub          = ProxyPublisher(       {self._cmd_topic: TwistStamped})

        self._odom_topic   = odometry_topic
        self._odom_sub     = ProxySubscriberCached({self._odom_topic:  Odometry})

        self._starting_odom = None


    def execute(self, userdata):


        #This checks the odom to get the current position, if the position is backed
        #up enough then the robot will finish the job
        if (self._sub.has_msg(self._odom_topic)):
            last_odom = self._sub.get_last_msg(self._odom_topic).pose.pose.position
            starting_odom_position = self._starting_odom.pose.pose.position

            distance_traveled = math.sqrt( (starting_odom_position.x - last_odom.x)**2.0 + (starting_odom_position.y - last_odom.y)**2.0)
            Logger.loginfo('distance traveled %f' %(distance_traveled))

            if (self._distance <= 0):
                if (distance_traveled >= (-1*self._distance - 0.1)):
                    return 'done'
            if (self._distance >= 0):
                if (distance_traveled >= (self._distance - 0.1)):
                    return 'done'


        if (self._done):
            # We have completed the state, and therefore must be blocked by autonomy level
            # Stop the robot, but and return the prior outcome
            ts = TwistStamped()
            ts.header.stamp = rospy.Time.now()
            self._pub.publish(self._cmd_topic, ts)
            return self._done

        if (rospy.Time.now() - self._start_time > self._target_time ):
            # If the time it takes to back up is longer than the desired time
            #constraints then will return the failure state
            return 'failed'

        # Normal operation
        self._twist.header.stamp = rospy.Time.now()
        self._pub.publish(self._cmd_topic, self._twist)
        return None

    def on_enter(self, userdata):
        # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
        if (self._sub.has_msg(self._odom_topic)):
            self._starting_odom = self._sub.get_last_msg(self._odom_topic)
        self._start_time = rospy.Time.now()
        self._done       = None # reset the completion flag
