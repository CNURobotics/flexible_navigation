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

import math
import time
import numpy as np
from copy import deepcopy
import rclpy
from rclpy.duration import Duration
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from functools import partial

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
from flexbe_core.proxy import ProxySubscriberCached
from flexbe_core.proxy import ProxyServiceCaller
from flexbe_core.proxy import ProxyActionClient

from geometry_msgs.msg import TwistStamped, Point, PointStamped, Vector3
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

class PurePursuitState(EventState):
    """
    This state calculates the twist required to follow a constant curvature arc to the pure pursuit intersection point.
    The command is published as a TwistStamped command based on parameters.

    If arc motion is used, the arc should be less than or equal to pi/2 radians.  Use multiple targets for longer arcs.

       -- desired_velocity     float     Desired velocity in m/s (default: 0.2)
       -- max_rotation_rate    float     Maximum rotation rate radians/s (default: 10.0)
       -- target_frame:        string    Coordinate frame of target point (default: 'map')
       -- target_x:            float     Target point x-coordinate (m)
       -- target_y:            float     Target point y-coordinate (m)
       -- target_type:         string    Desired motion ('line','arc') (default: 'line')
       -- lookahead_distance:  float     Lookahead distance (m) (default:  0.25)
       -- timeout              float     Transform timeout (seconds) (default: 0.08)
       -- recover_mode         bool      Recover path (typically on startup) (default: False)
       -- center_x:            float     Center point x-coordinate for circle defining arc motion (default: 0.0)
       -- center_y:            float     Center point y-coordinate for circle defining arc motion (default: 0.0)
       -- cmd_topic            string    topic name of the robot command (default: '/create_node/cmd_vel')
       -- odometry_topic:      string    topic of the iRobot Create sensor state (default:   '/create_node/odom'
       -- marker_topic:        string    topic of the RViz marker used for visualization (default: '/pure_pursuit_marker')
       -- marker_size:         float     Size of RViz marker used for target (default: 0.05)

       ># indice                int	 The index
       ># path			Path	 The path

       #> indice		int	 The index + 1
       #> plan			Path	 The path

       <= done                 Reached the end of target relevance
       <= continue		Continue following the path
       <= failed               A bumper was activated prior to completion
    """

    def __init__(self,  desired_velocity=0.2, max_rotation_rate=10.0,
                        target_frame='map', target_x=1.0, target_y=0.1, target_type='line',
                        lookahead_distance=0.25, timeout=0.08, recover_mode=False,
                        center_x=0.0, center_y=0.0,
                        cmd_topic='cmd_vel', odometry_topic='odom',
                        marker_topic='pure_pursuit_marker', marker_size=0.05):
        # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
        super(PurePursuitState, self).__init__(outcomes = ['done', 'continue', 'failed'],
                                                     input_keys = ['indice', 'plan'],
                                                     output_keys = ['indice', 'plan'])

        ProxyPublisher._initialize(PurePursuitState._node)
        ProxySubscriberCached._initialize(PurePursuitState._node)
        ProxyActionClient._initialize(PurePursuitState._node)
        ProxyServiceCaller._initialize(PurePursuitState._node)

        # Store state parameter for later use.
        self._twist                 = TwistStamped()
        self._twist.twist.linear.x  = desired_velocity
        self._twist.twist.angular.z = 0.0

        self._desired_velocity      = desired_velocity
        self._max_rotation_rate     = max_rotation_rate     # Used to clamp the rotation calculations


        self._current_position = PointStamped()
        self._current_position.header.stamp = self._node.get_clock().now().to_msg()
        self._current_position.header.frame_id = target_frame

        self._target = PointStamped()
        self._target.header.stamp = self._node.get_clock().now().to_msg()
        self._target.header.frame_id = target_frame
        self._target.point = Point(x=target_x, y=target_y, z=0.0)

        self._prior = PointStamped()
        self._prior.header.stamp = self._node.get_clock().now().to_msg()
        self._prior.header.frame_id = target_frame

        self._lookahead    = lookahead_distance
        self._recover_mode = recover_mode
        self._target_type  = target_type
        self._target_frame = target_frame
        if (self._target_type == 'arc'):
            self._radius = np.sqrt((center_x - target_x)**2 + (center_y - target_y)**2)

            Logger.loginfo('Using arc type with center=(%d, %d) target=(%d,%d) radius=%d'%(self._center.point.x,self._center.point.y,
                                                                                           self._target.point.x,self._target.point.y,
                                                                                           self._radius))


        self._odom_frame   = 'odom' # Update with the first odometry message
        self._robot_frame  = 'base_footprint'
        self._failed       = False
        self._timeout      = Duration(seconds=timeout) # transform timeout

        # The constructor is called when building the state machine, not when actually starting the behavior.
        # Thus, we cannot save the starting time now and will do so later.
        self._start_time = None

        self._return     = None # Track the outcome so we can detect if transition is blocked

        self._odom_topic   = odometry_topic
        self._marker_topic = marker_topic
        self._cmd_topic    = cmd_topic

        self._tf_buffer    = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, PurePursuitState._node)

        self._odom_sub     = ProxySubscriberCached({self._odom_topic:  Odometry})
        self._pub          = ProxyPublisher(       {self._cmd_topic: TwistStamped})

        if (self._marker_topic != ""):
            self._marker_pub   = ProxyPublisher({self._marker_topic: Marker})
        else:
            self._marker_pub = None

        self._marker       = Marker()
        self._marker.header.frame_id = self._target_frame
        self._marker.header.stamp = self._node.get_clock().now().to_msg()
        self._marker.ns     = "pure_pursuit_waypoints"
        self._marker.id     = int(target_x*1000000)+int(target_y*1000)
        self._marker.type   = Marker.SPHERE
        self._marker.action = Marker.ADD
        self._marker.pose.position.x = target_x
        self._marker.pose.position.y = target_y
        self._marker.pose.position.z = 0.0
        self._marker.pose.orientation.x = 0.0
        self._marker.pose.orientation.y = 0.0
        self._marker.pose.orientation.z = 0.0
        self._marker.pose.orientation.w = 1.0
        self._marker.scale.x = marker_size
        self._marker.scale.y = marker_size
        self._marker.scale.z = marker_size
        self._marker.color.a = 1.0  # Don't forget to set the alpha!
        self._marker.color.r = 0.0
        self._marker.color.g = 0.0
        self._marker.color.b = 1.0

        self._reference_marker = Marker()
        self._reference_marker.header.frame_id = self._target_frame
        self._reference_marker.header.stamp = self._node.get_clock().now().to_msg()
        self._reference_marker.ns = "pure_pursuit_reference"
        self._reference_marker.id = 1
        self._reference_marker.type   = Marker.SPHERE
        self._reference_marker.action = Marker.ADD
        self._reference_marker.pose.position.x = target_x
        self._reference_marker.pose.position.y = target_y
        self._reference_marker.pose.position.z = 0.0
        self._reference_marker.pose.orientation.x = 0.0
        self._reference_marker.pose.orientation.y = 0.0
        self._reference_marker.pose.orientation.z = 0.0
        self._reference_marker.pose.orientation.w = 1.0
        self._reference_marker.scale.x = marker_size*0.75
        self._reference_marker.scale.y = marker_size*0.75
        self._reference_marker.scale.z = marker_size*0.75
        self._reference_marker.color.a = 0.0  # Add, but make invisible at first
        self._reference_marker.color.r = 1.0
        self._reference_marker.color.g = 0.0
        self._reference_marker.color.b = 1.0

        self._local_target_marker = Marker()
        self._local_target_marker.header.frame_id = self._target_frame
        self._local_target_marker.header.stamp = self._node.get_clock().now().to_msg()
        self._local_target_marker.ns = "pure_pursuit_target"
        self._local_target_marker.id = 1
        self._local_target_marker.type   = Marker.SPHERE
        self._local_target_marker.action = Marker.ADD
        self._local_target_marker.pose.position.x = target_x
        self._local_target_marker.pose.position.y = target_y
        self._local_target_marker.pose.position.z = 0.0
        self._local_target_marker.pose.orientation.x = 0.0
        self._local_target_marker.pose.orientation.y = 0.0
        self._local_target_marker.pose.orientation.z = 0.0
        self._local_target_marker.pose.orientation.w = 1.0
        self._local_target_marker.scale.x = marker_size
        self._local_target_marker.scale.y = marker_size
        self._local_target_marker.scale.z = marker_size
        self._local_target_marker.color.a = 0.0  # Add, but make invisible at first
        self._local_target_marker.color.r = 1.0
        self._local_target_marker.color.g = 0.0
        self._local_target_marker.color.b = 1.0

    # Transform point into odometry frame
    def transformOdom(self, point):
        try:
            # Get transform
            return self._tf_buffer.transform(point, self._odom_frame, timeout=self._timeout, new_type = None)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            Logger.logwarn('Failed to get the transformation to odom frame\n%s' % str(e))
            self._failed = True
            return None
        except Exception as e:
            Logger.logwarn("Currently, unable to use Pure Pursuit due to TF2 issue #110 furthered described in CHANGELOG")
            Logger.logwarn('Failed to get the transformation to odom frame due to unknown error\n %s' % str(e) )
            self._failed = True
            return None

    # Transform point into map frame
    def transformMap(self, point):
        try:
            # Get transform
            return self._tf_buffer.transform(point, self._target_frame, timeout=self._timeout, new_type = None)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            Logger.logwarn('Failed to get the transformation to target_frame\n%s' % str(e))
            self._failed = True
            return None
        except Exception as e:
            Logger.logwarn("Currently, unable to use Pure Pursuit due to TF2 issue #110 furthered described in CHANGELOG")
            Logger.logwarn('Failed to get the transformation to target frame due to unknown error\n %s' % str(e) )
            self._failed = True
            return None

    # Transform point into robot body frame
    def transformBody(self, point):
        try:
            # Get transform
            return self._tf_buffer.transform(point, self._robot_frame, timeout=self._timeout, new_type = None)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            Logger.logwarn('Failed to get the transformation to robot body frame\n%s' % str(e))
            self._failed = True
            return None
        except:
            Logger.logwarn("Currently, unable to use Pure Pursuit due to TF2 issue #110 furthered described in CHANGELOG")
            Logger.logwarn('Failed to get the transformation to robot body frame due to unknown error\n' )
            self._failed = True
            return None

    def on_start(self):
        self._return = None # Clear completion flag

        # Wait for odometry message
        while (not self._odom_sub.has_msg(self._odom_topic)):
            Logger.logwarn('Waiting for odometry message from the robot ' )
            rate = PurePursuitState._node.create_rate(0.25)
            rate.sleep()

        self._last_odom = self._sub.get_last_msg(self._odom_topic)
        self._odom_frame = self._last_odom.header.frame_id
        Logger.loginfo('   odometry frame id <%s>' % (self._odom_frame))

        # Update the target transformation
        # self._target.header.stamp = self._last_odom.header.stamp
        while (self.transformOdom(self._target) is None):
            Logger.logwarn('Waiting for tf2_ros transformations to odometry frame to become available from the robot ' )
            rate = PurePursuitState._node.create_rate(0.25)
            rate.sleep()
            self._target.header.stamp = self._node.get_clock().now().to_msg()

        while (self.transformMap(self._last_odom) is None):
            Logger.logwarn('Waiting for tf2_ros transformations to map frame become available from the robot ' )
            rate = PurePursuitState._node.create_rate(0.25)
            rate.sleep()
            self._last_odom = self._sub.get_last_msg(self._odom_topic)

        point = PointStamped()
        point.header = self._last_odom.header
        point.point = self._last_odom.pose.pose.position
        self._current_position = self.transformMap(point)
        Logger.loginfo("Starting point = " + str(self._current_position.x) + " " + str(self._current_position.y))

        # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
        if (self._marker_pub):
            self._marker.action  = Marker.ADD
            self._marker.color.a = 1.0 # Set alpha otherwise the marker is invisible
            self._marker.color.r = 0.0
            self._marker.color.g = 0.0
            self._marker.color.b = 1.0 # Indicate this target is planned
            self._marker_pub.publish(self._marker_topic,self._marker)
            self._marker_pub.publish(self._marker_topic,self._reference_marker)
            self._marker_pub.publish(self._marker_topic,self._local_target_marker)
            self._marker.action           = Marker.MODIFY
            self._reference_marker.action = Marker.MODIFY
            self._reference_marker.color.a = 1.0 # Set alpha so it will become visible on next publish
            self._local_target_marker.action = Marker.MODIFY
            self._local_target_marker.color.a = 1.0 # Set alpha so it will become visible on next publish

    def on_enter(self, userdata):
        # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
        self._start_time = self._node.get_clock().now()
        self._return     = None  # reset the completion flag
        self._failed     = False # reset the failed flag

        if (self._marker_pub):
            self._marker.action  = Marker.MODIFY
            self._marker.color.a = 1.0
            self._marker.color.r = 0.0
            self._marker.color.g = 1.0 # Indicate this target is active
            self._marker.color.b = 0.0
            self. _marker_pub.publish(self._marker_topic,self._marker)

        if (userdata.indice > 0 and userdata.indice < len(userdata.plan.poses)):
            Logger.loginfo("   Access data for index %d" % (userdata.indice) )
            self._target.point = Point(x=userdata.plan.poses[userdata.indice].pose.position.x, y=userdata.plan.poses[userdata.indice].pose.position.y, z=0.0)
            self._prior.point  = Point(x=userdata.plan.poses[userdata.indice-1].pose.position.x, y=userdata.plan.poses[userdata.indice-1].pose.position.y, z=0.0)
            Logger.loginfo("  Moving toward  target %d =%f,%f  from prior=%f,%f" %
                    (userdata.indice, self._target.point.x, self._target.point.y, self._prior.point.x, self._prior.point.y))
        else:
            Logger.logerr("   Invalid index %d - cannot access the path points!" % (userdata.indice) )
            self._target.point = None
            self._prior.point  = None
            self._failed       = True
            self._return       = 'failed'

        # Increment the index for the next segment
        userdata.indice += 1

    def execute(self, userdata):
        # This method is called periodically while the state is active.
        # If no outcome is returned, the state will stay active.

        if (self._return):
            # We have completed the state, and therefore must be blocked by autonomy level
            # Stop the robot, but and return the prior outcome
            ts = TwistStamped()
            ts.header.stamp = self._node.get_clock().now().to_msg()
            self._pub.publish(self._cmd_topic, ts.twist)
            return self._return

        # Get the latest odometry data
        if (self._sub.has_msg(self._odom_topic)):
            self._last_odom = self._sub.get_last_msg(self._odom_topic)

        point = PointStamped()
        point.header = self._last_odom.header
        point.point = self._last_odom.pose.pose.position

        # Update the current pose
        self._current_position = self.transformMap(point)

        if (self._current_position is None):
            Logger.loginfo("Could not get transform position")
            return 'failed'

        if (self._failed):
             self._return = 'failed'
             return 'failed'

        # Transform the target points into the current odometry frame
        self._target.header.stamp = self._last_odom.header.stamp
        local_target = self._target; #self.transformOdom(self._target)

        # If target point is withing lookahead distance then we are done
        dr = np.sqrt((local_target.point.x - self._current_position.point.x)**2 + (local_target.point.y - self._current_position.point.y)**2)
        if (dr < self._lookahead):
            Logger.loginfo(' %s  Lookahead circle is past target - done! target=(%f, %f, %f) robot=(%f,%f, %f)  dr=%f lookahead=%f ' %
                    (self.name, local_target.point.x,local_target.point.y,local_target.point.z,
                     self._current_position.point.x, self._current_position.point.y, self._current_position.point.z,
                     dr,self._lookahead))
            if (userdata.indice == len(userdata.plan.poses)-1):
                self._return = 'done'
                return 'done'
            else:
                self._return = 'continue'
                return 'continue'

        # Transform the prior target point into the current odometry frame
        self._prior.header.stamp = self._last_odom.header.stamp
        local_prior = self._prior #self.transformOdom(self._prior)
        if (self._failed):
             self._return = 'failed'
             return 'failed'

        # Assume we can go the desired velocity
        self._twist.twist.linear.x = self._desired_velocity

        lookahead = None
        lookahead = self.calculateLineTwist(local_prior, local_target)

        if (lookahead is None):
             # Did not get a lookahead point so we either failed, completed segment, or are deliberately holding the prior velocity (to recover from minor perturbations)
             if (self._return is not None):
                 Logger.logerr("No lookahead!")
             return self._return # return what was set (either 'failed' or 'done')

        # Sanity check the rotation rate
        if (math.fabs(self._twist.twist.angular.z) > self._max_rotation_rate):
            self._twist.twist.linear.x  = self._desired_velocity*self._max_rotation_rate/math.fabs(self._twist.twist.angular.z) # decrease the speed
            self._twist.twist.angular.z = math.copysign(self._max_rotation_rate, self._twist.twist.angular.z)

        # Normal operation - publish the latest calculated twist
        self._twist.header.stamp = self._node.get_clock().now().to_msg()
        self._pub.publish(self._cmd_topic, self._twist.twist)

        if (self._marker_pub):
            self._marker_pub.publish(self._marker_topic,self._reference_marker)
            self._marker_pub.publish(self._marker_topic,self._local_target_marker)

        return None

    def on_exit(self, userdata):
        # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
        elapsed_time = self._node.get_clock().now() - self._start_time
        if (self._marker_pub):
            self._marker.color.a = 1.0 # Don't forget to set the alpha!
            self._marker.color.r = 0.8 # Indicate this target is no longer active
            self._marker.color.g = 0.0
            self._marker.color.b = 0.0
            self. _marker_pub.publish(self._marker_topic,self._marker)

    # Method to calculate the lookahead point given line segment from prior to target
    def calculateLineTwist(self, local_prior, local_target):

        # Define a line segment from prior to the target (assume 2D)
        pv = Vector3(x=local_target.point.x - local_prior.point.x, y=local_target.point.y - local_prior.point.y, z=0.0)
        qv = Vector3(x=local_prior.point.x  - self._current_position.point.x, y=local_prior.point.y  - self._current_position.point.y, z=0.0)

        # Find intersection of line segment with lookahead circle centered at the  robot
        a = pv.x*pv.x + pv.y*pv.y #
        b = 2.0*(qv.x*pv.x + qv.y*pv.y)
        c = (qv.x*qv.x + qv.y*qv.y) - self._lookahead*self._lookahead

        if (a < 0.001):
            Logger.logerr(' %s  Invalid prior and target for line: target=(%f, %f, %f) prior=(%f, %f, %f) robot=(%f,%f, %f)  pv=(%f,%f) qv=(%f,%f) a=%f b=%f c=%f ' %
                 (self.name, local_target.point.x,local_target.point.y,local_target.point.z,
                  local_prior.point.x ,local_prior.point.y ,local_prior.point.z,
                  self._current_position.point.x, self._current_position.point.y, self._current_position.point.z,
                  pv.x, pv.y,qv.x,qv.y,a,b,c))
            self._return = 'failed'
            return None

        discrim = b*b - 4*a*c
        if (discrim < 0.0):
             # No intersection - this should not be unless bad start or localization perturbation
            Logger.logwarn(' %s  No path recovery - no intersection for line: target=(%f, %f, %f) prior=(%f, %f, %f) robot=(%f,%f, %f)  pv=(%f,%f) qv=(%f,%f) a=%f b=%f c=%f discrim=%f ' %
                 (self.name, local_target.point.x,local_target.point.y,local_target.point.z,
                  local_prior.point.x ,local_prior.point.y ,local_prior.point.z,
                  self._current_position.point.x, self._current_position.point.y, self._current_position.point.z,
                  pv.x, pv.y,qv.x,qv.y,a,b,c,discrim))
            self._return = 'failed'
            return None
        else:
            # solve quadratic equation for intersection points
            sqd = math.sqrt(discrim)
            t1 = (-b - sqd)/(2*a)  # min value
            t2 = (-b + sqd)/(2*a)  # max value
            if (t2 < t1):
                Logger.logwarn(' %s  Say what! t1=%f t2=%f sqd=%f target=(%f, %f, %f) prior=(%f, %f, %f) robot=(%f,%f, %f)  pv=(%f,%f) qv=(%f,%f) a=%f b=%f c=%f discrim=%f ' %
                        (self.name, t1, t2, sqd,
                         local_target.point.x,local_target.point.y,local_target.point.z,
                         local_prior.point.x ,local_prior.point.y ,local_prior.point.z,
                         self._current_position.point.x, self._current_position.point.y, self._current_position.point.z,
                         pv.x, pv.y,qv.x,qv.y,a,b,c,discrim))
                self._return = 'failed'
                return None

            if (t2 < 0.0):
                # all intersections are behind the segment - this should not be unless bad start or localization perturbation
                if (t2 > -0.1):
                    # likely due to localization perturbation
                    Logger.logwarn(' %s  Circle is before segment - continue prior motion! t1=%f t2=%f sqd=%f target=(%f, %f, %f) prior=(%f, %f, %f) robot=(%f,%f, %f)  pv=(%f,%f) qv=(%f,%f) a=%f b=%f c=%f discrim=%f ' %
                        (self.name, t1, t2, sqd,
                         local_target.point.x,local_target.point.y,local_target.point.z,
                         local_prior.point.x ,local_prior.point.y ,local_prior.point.z,
                         self._current_position.point.x, self._current_position.point.y, self._current_position.point.z,
                         pv.x, pv.y,qv.x,qv.y,a,b,c,discrim))
                    return None
                else:
                    Logger.logwarn(' %s  Circle is before segment! t1=%f t2=%f sqd=%f target=(%f, %f, %f) prior=(%f, %f, %f) robot=(%f,%f, %f)  pv=(%f,%f) qv=(%f,%f) a=%f b=%f c=%f discrim=%f ' %
                        (self.name, t1, t2, sqd,
                         local_target.point.x,local_target.point.y,local_target.point.z,
                         local_prior.point.x ,local_prior.point.y ,local_prior.point.z,
                         self._current_position.point.x, self._current_position.point.y, self._current_position.point.z,
                         pv.x, pv.y,qv.x,qv.y,a,b,c,discrim))
                    self._return = 'failed'
                    return None
            elif (t1 > 1.0):
                # all intersections are past the segment
                Logger.loginfo(' %s  Circle is past segment - done! t1=%f t2=%f sqd=%f target=(%f, %f, %f) prior=(%f, %f, %f) robot=(%f,%f, %f)  pv=(%f,%f) qv=(%f,%f) a=%f b=%f c=%f discrim=%f ' %
                        (self.name, t1, t2, sqd,
                         local_target.point.x,local_target.point.y,local_target.point.z,
                         local_prior.point.x ,local_prior.point.y ,local_prior.point.z,
                         self._current_position.point.x, self._current_position.point.y, self._current_position.point.z,
                         pv.x, pv.y,qv.x,qv.y,a,b,c,discrim))
                self._return = 'done'
                return None
            elif (t1 < 0.0 and t2 > 1.0):
                # Segment is contained inside the lookahead circle
                Logger.loginfo(' %s  Circle contains segment - move along! t1=%f t2=%f sqd=%f target=(%f, %f, %f) prior=(%f, %f, %f) robot=(%f,%f, %f)  pv=(%f,%f) qv=(%f,%f) a=%f b=%f c=%f discrim=%f ' %
                        (self.name, t1, t2, sqd,
                         local_target.point.x,local_target.point.y,local_target.point.z,
                         local_prior.point.x ,local_prior.point.y ,local_prior.point.z,
                         self._current_position.point.x, self._current_position.point.y, self._current_position.point.z,
                         pv.x, pv.y,qv.x,qv.y,a,b,c,discrim))
                self._return = 'done'
                return None
            elif (t2 > 1.0):
                # The lookahead circle extends beyond the target point - we are finished here
                Logger.loginfo(' %s  Circle extends past segment - done! t1=%f t2=%f sqd=%f target=(%f, %f, %f) prior=(%f, %f, %f) robot=(%f,%f, %f)  pv=(%f,%f) qv=(%f,%f) a=%f b=%f c=%f discrim=%f ' %
                        (self.name, t1, t2, sqd,
                         local_target.point.x,local_target.point.y,local_target.point.z,
                         local_prior.point.x ,local_prior.point.y ,local_prior.point.z,
                         self._current_position.point.x, self._current_position.point.y, self._current_position.point.z,
                         pv.x, pv.y,qv.x,qv.y,a,b,c,discrim))
                self._return = 'done'
                return None
            else:
                # This is the normal case
                # Must be line segment
                control = deepcopy(local_prior)
                control.point.x = control.point.x + t2*pv.x # Control point in the odometry frame
                control.point.y = control.point.y + t2*pv.y # Control point in the odometry frame
                control.point.z = control.point.z + t2*pv.z # Control point in the odometry frame
                self._reference_marker.pose.position = control.point
                self._local_target_marker.pose.position = local_target.point

                control_robot = self.transformBody(control)

                curvature = 2.0*control_robot.point.y/(self._lookahead*self._lookahead)
                self._twist.twist.angular.z  = curvature*self._desired_velocity
                return control_robot

        return None
