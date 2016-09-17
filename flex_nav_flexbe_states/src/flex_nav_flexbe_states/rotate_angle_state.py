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

class RotateAngleState(EventState):
    '''
    Rotates the robot a full 360 degrees in the given time defined by userdata
    -- target_time      float       Time which needs to have passed since the behavior starte
    -- odometry_topic   string      topic of the iRobot Create sensor state (default:   '/create_node/odom')
    -- cmd_topic        string      Topic name of the robot command (default: '/create_node/cmd_vel')
    <= done                         Given time has passed.
    '''
    def __init__(self, target_time, cmd_topic='/create_node/cmd_vel', odometry_topic='/create_node/odom'):
        super(RotateAngleState, self).__init__(outcomes = ['done'])
        self._target_time           = rospy.Duration(target_time)
        self._twist                 = TwistStamped()
        self._twist.twist.linear.x  = 0
        self._twist.twist.angular.z = (6.28319 / target_time)

        self._cmd_topic    = cmd_topic
        self._pub          = ProxyPublisher(       {self._cmd_topic: TwistStamped})
        self._start_time = None
        self._done       = None # Track the outcome so we can detect if transition is blocked

    def execute(self, userdata):
        if (rospy.Time.now() - self._start_time > self._target_time):
            #The robot will twist in place until the desired time has elapsed
            #Once it has the it will return done
            self._done = 'done'
            return 'done'

        self._twist.header.stamp = rospy.Time.now()
        self._pub.publish(self._cmd_topic, self._twist)
        return None

    def on_enter(self, userdata):
        self._start_time = rospy.Time.now()
        self._done       = None # reset the completion flag
