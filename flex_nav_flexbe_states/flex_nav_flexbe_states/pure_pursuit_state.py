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

import math
import numpy as np
import traceback

from copy import deepcopy
import rclpy
from rclpy.duration import Duration
import tf2_ros

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
from flexbe_core.proxy import ProxyTransformListener

from geometry_msgs.msg import Twist, TwistStamped, Point, PointStamped, Vector3
from visualization_msgs.msg import Marker, MarkerArray


class PurePursuitState(EventState):
    """
    Calculate the twist required to follow a path using pure pursuit.

    Calculates the twist required to follow a constant curvature arc
    to the pure pursuit intersection point based on linear interpolation between two way points.

    The command is published as a TwistStamped command based on parameters.

    If arc motion is used, the arc should be less than or equal to pi/2 radians.  Use multiple targets for longer arcs.

       -- desired_velocity     float     Desired velocity in m/s (default: 0.2)
       -- max_rotation_rate    float     Maximum rotation rate radians/s (default: 10.0)
       -- lookahead_distance:  float     Lookahead distance (m) (default:  0.25)
       -- timeout              float     Transform timeout (seconds) (default: 0.5)
       -- cmd_topic            string    topic name of the robot command (default: 'cmd_vel')
       -- marker_topic:        string    topic of the RViz marker used for visualization (default: 'pure_pursuit_markers')
       -- marker_size:         float     Size of RViz marker used for target (default: 0.05)
       -- cmd_topic_stamped    string    optional topic name of the stamped robot command (default: '')

       ># indice                int      The index
       ># path                  Path     The path

       #> indice                int      The index + 1
       #> plan                  Path     The path

       <= done          Reached the end of the whole path
       <= continue      Continue reached end of one segment in path, exit to log with intent to loop back to followthe path
       <= failed        Could not process segment properly due to transforms or other issues
    """

    def __init__(self, desired_velocity=0.2, max_rotation_rate=10.0,
                 lookahead_distance=0.25, timeout=0.5,
                 cmd_topic='cmd_vel',
                 marker_topic='pure_pursuit_markers', marker_size=0.05,
                 cmd_topic_stamped=''):
        # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
        super().__init__(outcomes=['done', 'continue', 'failed'],
                         input_keys=['indice', 'plan'],
                         output_keys=['indice', 'plan'])

        ProxyPublisher.initialize(PurePursuitState._node)
        ProxyTransformListener.initialize(PurePursuitState._node)

        # Store state parameter for later use.
        self._desired_velocity  = desired_velocity
        self._max_rotation_rate = max_rotation_rate  # Used to clamp the rotation calculations

        self._target = None

        self._prior = None

        self._lookahead    = lookahead_distance

        self._robot_frame  = 'base_footprint'

        self._target_frame = 'map'  # Will update with path data

        self._failed       = False
        self._timeout      = Duration(seconds=timeout)  # transform timeout

        # The constructor is called when building the state machine, not when actually starting the behavior.
        # Thus, we cannot save the starting time now and will do so later.
        self._start_time = None

        self._return     = None  # Track the outcome so we can detect if transition is blocked

        self._tf_listener = ProxyTransformListener()
        self._tf_buffer   = self._tf_listener.buffer

        # Command data
        self._twist = Twist()
        self._twist.linear.x  = desired_velocity
        self._twist.angular.z = 0.0

        self._pub = ProxyPublisher()
        if isinstance(cmd_topic, str) and len(cmd_topic) != 0:
            self._cmd_topic = cmd_topic
            self._pub.createPublisher(cmd_topic, Twist)
        else:
            self._cmd_topic = None

        if isinstance(cmd_topic_stamped, str) and len(cmd_topic_stamped) != 0:
            self._twist_stamped  = TwistStamped()
            self._twist_stamped.twist.linear.x  = desired_velocity
            self._twist_stamped.twist.angular.z = 0.0
            self._cmd_topic_stamped = cmd_topic_stamped
            self._pub.createPublisher(cmd_topic_stamped, TwistStamped)
        else:
            self._twist_stamped  = None
            self._cmd_topic_stamped = None

        if self._cmd_topic is None and self._cmd_topic_stamped is None:
            Logger.logerr("Must define at least one cmd or cmd_stamped publishing topic")
        if self._cmd_topic == self._cmd_topic_stamped:
            Logger.logerr("Must define different names for cmd_topic and cmd_topic_stamped topics")

        assert self._cmd_topic or self._cmd_topic_stamped, "Must define at least one cmd publishing topic"
        assert self._cmd_topic != self._cmd_topic_stamped, "Must be different topic names!"

        self._marker_topic = None
        if marker_topic and len(marker_topic) > 0:
            self._marker_topic = marker_topic
            self._pub.createPublisher(self._marker_topic, MarkerArray)

            # Marker point that we are steering toward along the intersection
            self._reference_marker = Marker()
            self._reference_marker.header.frame_id = self._target_frame
            self._reference_marker.header.stamp = self._node.get_clock().now().to_msg()
            self._reference_marker.ns = "pure_pursuit_reference"
            self._reference_marker.id = 1
            self._reference_marker.type   = Marker.SPHERE
            self._reference_marker.action = Marker.ADD
            self._reference_marker.pose.position.x = 0.0
            self._reference_marker.pose.position.y = 0.0
            self._reference_marker.pose.position.z = 0.0
            self._reference_marker.pose.orientation.x = 0.0
            self._reference_marker.pose.orientation.y = 0.0
            self._reference_marker.pose.orientation.z = 0.0
            self._reference_marker.pose.orientation.w = 1.0
            self._reference_marker.scale.x = marker_size * 0.75
            self._reference_marker.scale.y = marker_size * 0.75
            self._reference_marker.scale.z = marker_size * 0.75
            self._reference_marker.color.a = 0.0  # Add, but make invisible at first
            self._reference_marker.color.r = 1.0
            self._reference_marker.color.g = 0.0
            self._reference_marker.color.b = 1.0

            # End point of current segment
            self._local_target_marker = Marker()
            self._local_target_marker.header.frame_id = self._target_frame
            self._local_target_marker.header.stamp = self._node.get_clock().now().to_msg()
            self._local_target_marker.ns = "pure_pursuit_target"
            self._local_target_marker.id = 1
            self._local_target_marker.type   = Marker.SPHERE
            self._local_target_marker.action = Marker.ADD
            self._local_target_marker.pose.position.x = 0.0
            self._local_target_marker.pose.position.y = 0.0
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
            self._marker_array = MarkerArray()
            self._marker_array.markers.append(self._reference_marker)
            self._marker_array.markers.append(self._local_target_marker)

        else:
            self._reference_marker = None
            self._local_target_marker = None
            self._marker_array = None

    def transform_frame(self, point, frame_id):
        """
        Transform point into map frame.

        @param point - location with header defining current frame_id
        @param target_frame - which frame to represent
        @return timestamped point in new frame given latest available transform
        """
        try:
            # Get latest transform
            #       Note:  _tf_buffer.transform() not working due to https://github.com/ros2/geometry2/issues/110
            #               return self._tf_buffer.transform(point, frame_id, timeout=self._timeout, new_type = None)
            ts = self._tf_buffer.lookup_transform(frame_id,  # to new frame
                                                  point.header.frame_id,  # from frame
                                                  time=rclpy.time.Time())  # Just grab the latest transform that is available

            t_ts = self._make_transform(ts.transform.translation, ts.transform.rotation)
            p_ts = np.dot(t_ts, np.array([point.point.x, point.point.y, point.point.z, 1.0]).T)

            point = PointStamped(header=ts.header, point=Point(x=p_ts[0], y=p_ts[1], z=p_ts[2]))
            return point, t_ts

        except (tf2_ros.ExtrapolationException) as e:
            Logger.logwarn(f'Failed to get the transformation from {point.header.frame_id} to '
                           f'target_frame {frame_id} at t={point.header.stamp} within '
                           f'{self._timeout} s now={self._node.get_clock().now().to_msg()}\n{type(e)} {str(e)}')
            self._failed = True
            return None

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException) as e:
            Logger.logwarn(f'Failed to get the transformation from {point.header.frame_id} to '
                           f'target_frame {frame_id} at t={point.header.stamp} within '
                           f'{self._timeout} s\n{type(e)} {str(e)}')
            self._failed = True
            return None
        except (TypeError, tf2_ros.buffer_interface.TypeException) as e:
            # https://github.com/ros2/geometry2/issues/110
            msg = (f'Failed to get the transformation from {point.header.frame_id} to {frame_id} frame due to '
                   f' type conversion error\n {type(e)} {str(e)}')
            msg = msg.replace('<', "[")  # Avoid string format issues with logger
            msg = msg.replace('>', "]")
            Logger.logwarn(msg)
            Logger.loginfo("See note in CHANGELOG")
            self._failed = True
            return None
        except Exception as e:
            msg = (f'Failed to get the transformation from {point.header.frame_id} to {frame_id} frame '
                   f'due to unknown error\n {type(e)} {str(e)}')
            msg = msg.replace('<', "[")  # Avoid string format issues with logger
            msg = msg.replace('>', "]")
            Logger.logwarn(msg)
            trace = traceback.format_exc()
            Logger.localinfo(' --------------------- Trace ------------------------------')
            Logger.localinfo(f''' Trace: {trace.replace("%", "%%")}''')
            Logger.localinfo(' --------------------- Trace ------------------------------')
            self._failed = True
            return None

    def on_start(self):
        # This method is called when the state becomes active
        self._return = None  # Clear completion flag
        self._start_time = self._node.get_clock().now()
        self._failed = False

    def on_enter(self, userdata):
        # This method is called when the state becomes active,
        # i.e. a transition from another state to this one is taken.
        self._start_time = self._node.get_clock().now()
        self._return     = None  # reset the completion flag
        self._failed     = False  # reset the failed flag

        if 'indice' not in userdata or 'plan' not in userdata or len(userdata.plan.poses) < 1:
            Logger.logerr("Require starting indice and valid plan via userdata")
            self._target.point = None
            self._prior.point  = None
            self._failed       = True
            self._return       = 'failed'
            return self._return

        if userdata.indice > 0 and userdata.indice < len(userdata.plan.poses):

            # Match target frame to the given path that we will follow
            self._target_frame = userdata.plan.header.frame_id
            self._target = PointStamped()
            self._target.header.frame_id = self._target_frame  # path point headers are not always set
            self._prior = PointStamped()
            self._prior.header.frame_id = self._target_frame

            self._target.point = Point(x=userdata.plan.poses[userdata.indice].pose.position.x,
                                       y=userdata.plan.poses[userdata.indice].pose.position.y, z=0.0)
            self._prior.point  = Point(x=userdata.plan.poses[userdata.indice - 1].pose.position.x,
                                       y=userdata.plan.poses[userdata.indice - 1].pose.position.y, z=0.0)

        else:
            Logger.logerr("   Invalid index %d - cannot access any valid path points!" % (userdata.indice))
            self._target = None
            self._prior  = None
            self._failed = True
            self._return = 'failed'
            return self._return

        # Confirm transformations available
        cnt = 0
        rate = None
        target_transform = self.transform_frame(self._target, self._robot_frame)
        if target_transform is None:
            rate = PurePursuitState._node.create_rate(0.01)
            while (target_transform is None and cnt < 100):

                if cnt & 10 == 0:
                    Logger.logwarn(f'[{self._name}] Waiting for tf2_ros transformations from '
                                   f'target frame {self._target.header.frame_id} to '
                                   f'robot frame {self._robot_frame} available from the robot ')
                rate.sleep()
                target_transform = self.transform_frame(self._target, self._target_frame)
                cnt += 1
            self._node.destroy_rate(rate)
            if cnt == 100 and target_transform is None:
                Logger.logerr("[%s] Transform from %s to %s is not available - failed!"
                              % (self._name, self._target_frame, self._robot_frame))
                self._target = None
                self._prior  = None
                self._failed = True
                self._return = 'failed'
                return self._return

        local_target = target_transform[0]  # Just need the point

        delta = self._node.get_clock().now() - rclpy.time.Time.from_msg(local_target.header.stamp)
        if delta > self._timeout:
            Logger.logerr("[%s] Transform from %s to %s is too old delta=%ld ns ts=%s now=%s - failed!"
                          % (self._name, self._target_frame, self._robot_frame, delta.nanoseconds,
                             str(local_target.header.stamp), str(self._node.get_clock().now().to_msg())))
            self._target = None
            self._prior  = None
            self._failed = True
            self._return = 'failed'
            return self._return

        self._failed = False  # reset the failed flag from transform lookup
        if self._marker_topic:
            self._local_target_marker.header.frame_id = self._target_frame
            self._local_target_marker.color.a = 1.0  # Set alpha otherwise the marker is invisible
            self._local_target_marker.color.r = 0.0
            self._local_target_marker.color.g = 1.0  # active point
            self._local_target_marker.color.b = 0.0
            self._local_target_marker.pose.position = self._target.point
            self._reference_marker.header.frame_id = self._target_frame
            self._reference_marker.color.a = 1.0  # Set alpha otherwise the marker is invisible
            self._reference_marker.color.r = 0.0
            self._reference_marker.color.g = 1.0  # active point
            self._reference_marker.color.b = 0.0
            self._reference_marker.pose.position = self._prior.point  # temporary unit first calc in execute
            self._pub.publish(self._marker_topic, self._marker_array)

        self._node.get_logger().info("[%s]  Moving toward  target %d =%f,%f  from prior=%f,%f in frame %s"
                                     % (self._name, userdata.indice, self._target.point.x, self._target.point.y,
                                        self._prior.point.x, self._prior.point.y, self._target_frame))

    def execute(self, userdata):
        # This method is called periodically while the state is active.
        # If no outcome is returned, the state will stay active.

        if (self._return):
            # We have completed the state, and therefore must be blocked by autonomy level
            # Stop the robot, but and return the prior outcome
            if self._cmd_topic:
                self._pub.publish(self._cmd_topic, Twist())

            if self._cmd_topic_stamped:
                ts = TwistStamped()  # Zero twist to stop if blocked
                ts.header.stamp = self._node.get_clock().now().to_msg()  # update the time stamp
                self._pub.publish(self._cmd_topic_stamped, ts)

            return self._return

        self._node.get_logger().debug("[%s]  Moving toward  target %d =%f,%f  from prior=%f,%f in frame %s"
                                      % (self._name, userdata.indice, self._target.point.x, self._target.point.y,
                                         self._prior.point.x, self._prior.point.y, self._target_frame))

        # Update the current target pose into robot frame using the latest available transform
        self._node.get_logger().debug(f"target point {userdata.indice} (map) = {str(self._target)}!")
        target_transform = self.transform_frame(self._target, self._robot_frame)

        if target_transform is None or self._failed:
            Logger.loginfo("Could not get transformed target position")
            self._return = 'failed'
            return 'failed'

        local_target, current_transform = target_transform
        self._node.get_logger().debug(f"target point (local) = {str(local_target)}!")

        delta = self._node.get_clock().now() - rclpy.time.Time.from_msg(local_target.header.stamp)
        if delta > self._timeout:
            Logger.logerr("[%s] Transform of target from %s to %s is too old "
                          "delta=%ld ts=%s now=%s - failed!"
                          % (self._name, self._target_frame, self._robot_frame,
                             delta.nanoseconds, str(local_target.header.stamp),
                             str(self._node.get_clock().now().to_msg())))
            self._failed = True
            self._return = 'failed'
            return self._return

        # If target point is within lookahead distance then we are done with this segment
        dr = local_target.point.x**2 + local_target.point.y**2
        if dr < self._lookahead**2:
            self._node.get_logger().info(' %s  Lookahead circle is past target - done segment! '
                                         'local target=(%f, %f, %f) dr=%f lookahead=%f '
                                         % (self.name,
                                            local_target.point.x, local_target.point.y, local_target.point.z,
                                            dr, self._lookahead))
            if userdata.indice == (len(userdata.plan.poses) - 1):
                self._return = 'done'
                return 'done'
            else:
                # We deactivate this state to allow for external logging and monitoring
                # Need loop back to continue navigating along path
                self._return = 'continue'
                return 'continue'

        local_prior = deepcopy(local_target)  # header is same
        p_ts = np.dot(current_transform,
                      np.array([self._prior.point.x, self._prior.point.y, self._prior.point.z, 1.0]).T)
        local_prior.point.x = p_ts[0]
        local_prior.point.y = p_ts[1]
        local_prior.point.z = p_ts[2]

        # Assume we can go the desired velocity
        self._twist.linear.x = self._desired_velocity

        # Given line segment in robot body frame of reference
        lookahead = self.calculate_line_twist(local_prior, local_target)

        if (lookahead is None):
            # Did not get a lookahead point so we either failed, completed segment, or
            # are deliberately holding the prior velocity (to recover from minor perturbations)
            self._node.get_logger().info(f"No lookahead at index={userdata.indice} of {self._name} "
                                         f"return='{self._return}'!")
            return self._return  # return what was set (either 'failed', 'continue', or 'done')

        # Sanity check the rotation rate
        if (math.fabs(self._twist.angular.z) > self._max_rotation_rate):
            self._twist.linear.x  = (self._desired_velocity * self._max_rotation_rate
                                     / math.fabs(self._twist.angular.z))  # decrease the speed
            self._twist.angular.z = math.copysign(self._max_rotation_rate, self._twist.angular.z)

        # Normal operation - publish the latest calculated twist
        if self._cmd_topic:
            self._pub.publish(self._cmd_topic, self._twist)

        if self._cmd_topic_stamped:
            self._twist_stamped.header.stamp = self._node.get_clock().now().to_msg()  # update the time stamp
            self._twist_stamped.twist.linear.x = self._twist.linear.x
            self._twist_stamped.twist.angular.z = self._twist.angular.z
            self._pub.publish(self._cmd_topic_stamped, self._twist_stamped)

        if self._marker_topic:
            self._pub.publish(self._marker_topic, self._marker_array)

        return None  # Continue along this segment

    def on_exit(self, userdata):
        # This method is called when the state transitions to another state
        # NOTE: We are NOT stopping command on regular 'done' to allow for continuous transition
        # to a following segment. It is up to user issue stop command

        if self._return == 'continue':
            # Increment the index for the next segment
            userdata.indice += 1

        if self._start_time is not None and (self._return == 'failed' or self._manual_transition_requested == 'failed'):
            Logger.logerr(' %s   Emergency stop if failed when active ' % (self.name))
            if self._cmd_topic:
                self._pub.publish(self._cmd_topic, Twist())

            if self._cmd_topic_stamped:
                ts = TwistStamped()  # Zero twist to stop if blocked
                ts.header.stamp = self._node.get_clock().now().to_msg()  # update the time stamp
                self._pub.publish(self._cmd_topic_stamped, ts)

        if self._marker_topic:
            # Make invisible on exit
            self._reference_marker.color.a = 0.0  # Don't forget to set the alpha on reentry!
            self._reference_marker.color.r = 1.0  # Inactive point
            self._reference_marker.color.g = 0.0
            self._reference_marker.color.b = 0.0
            self._local_target_marker.color.a = 0.0  # Don't forget to set the alpha!
            self._pub.publish(self._marker_topic, self._marker_array)

        self._start_time = None
        self._target = None
        self._prior = None

    def on_stop(self):
        if self._start_time is not None:
            Logger.logerr(' %s   Emergency stop if stopped when active ' % (self.name))
            if self._cmd_topic:
                self._pub.publish(self._cmd_topic, Twist())

            if self._cmd_topic_stamped:
                ts = TwistStamped()  # Zero twist to stop if blocked
                ts.header.stamp = self._node.get_clock().now().to_msg()  # update the time stamp
                self._pub.publish(self._cmd_topic_stamped, ts)
            self._start_time = None

    def on_pause(self):
        if self._start_time is not None:
            Logger.logerr(' %s   Emergency stop if paused when active ' % (self.name))
            if self._cmd_topic:
                self._pub.publish(self._cmd_topic, Twist())

            if self._cmd_topic_stamped:
                ts = TwistStamped()  # Zero twist to stop if blocked
                ts.header.stamp = self._node.get_clock().now().to_msg()  # update the time stamp
                self._pub.publish(self._cmd_topic_stamped, ts)
            self._start_time = None

    # Method to calculate the lookahead point given line segment from prior to target
    def calculate_line_twist(self, local_prior, local_target):

        # Define a line segment from prior to the target (assume 2D)
        pv = Vector3(x=local_target.point.x - local_prior.point.x,
                     y=local_target.point.y - local_prior.point.y, z=0.0)

        # Vector from robot body reference to first point
        qv = Vector3(x=local_prior.point.x, y=local_prior.point.y, z=0.0)

        # Find intersection of line segment with lookahead circle centered at the  robot
        a = pv.x * pv.x + pv.y * pv.y
        b = 2.0 * (qv.x * pv.x + qv.y * pv.y)
        c = (qv.x * qv.x + qv.y * qv.y) - self._lookahead * self._lookahead

        if (a < 0.001):
            Logger.logerr(' %s  Invalid prior and target for line in %s frame: target=(%f, %f, %f)'
                          ' prior=(%f, %f, %f) pv=(%f,%f) qv=(%f,%f) a=%f b=%f c=%f '
                          % (self.name, local_target.header.frame_id,
                             local_target.point.x, local_target.point.y, local_target.point.z,
                             local_prior.point.x, local_prior.point.y, local_prior.point.z,
                             pv.x, pv.y, qv.x, qv.y, a, b, c))
            self._return = 'failed'
            return None

        discrim = b * b - 4 * a * c
        if (discrim < 0.0):
            # No intersection - this should not be unless bad start or localization perturbation
            Logger.logwarn(' %s  No path recovery - no intersection for line in %s frame: '
                           'target=(%f, %f, %f) prior=(%f, %f, %f) pv=(%f,%f) qv=(%f,%f) '
                           'a=%f b=%f c=%f discrim=%f '
                           % (self.name, local_target.header.frame_id,
                              local_target.point.x, local_target.point.y, local_target.point.z,
                              local_prior.point.x, local_prior.point.y, local_prior.point.z,
                              pv.x, pv.y, qv.x, qv.y, a, b, c, discrim))
            self._return = 'failed'
            return None
        else:
            # solve quadratic equation for intersection points
            sqd = math.sqrt(discrim)
            t1 = (-b - sqd) / (2 * a)  # min value
            t2 = (-b + sqd) / (2 * a)  # max value
            if (t2 < t1):
                Logger.logwarn(' %s  Say what! t1=%f t2=%f sqd=%f for line in %s frame: '
                               'target=(%f, %f, %f) prior=(%f, %f, %f) pv=(%f,%f) qv=(%f,%f) '
                               'a=%f b=%f c=%f discrim=%f '
                               % (self.name, t1, t2, sqd, local_target.header.frame_id,
                                  local_target.point.x, local_target.point.y, local_target.point.z,
                                  local_prior.point.x, local_prior.point.y, local_prior.point.z,
                                  pv.x, pv.y, qv.x, qv.y, a, b, c, discrim))
                self._return = 'failed'
                return None

            if (t2 < 0.0):
                # all intersections are behind the segment - this should not be unless bad start or localization perturbation
                if (t2 > -0.1):
                    # likely due to localization perturbation
                    Logger.logwarn(' %s  Circle is before segment - continue prior motion for '
                                   'line in %s frame: t1=%f t2=%f sqd=%f target=(%f, %f, %f) '
                                   'prior=(%f, %f, %f) pv=(%f,%f) qv=(%f,%f) a=%f b=%f c=%f discrim=%f '
                                   % (self.name, t1, t2, sqd, local_target.header.frame_id,
                                      local_target.point.x, local_target.point.y, local_target.point.z,
                                      local_prior.point.x, local_prior.point.y, local_prior.point.z,
                                      pv.x, pv.y, qv.x, qv.y, a, b, c, discrim))
                    return None
                else:
                    Logger.logwarn(' %s  Circle is before segment! t1=%f t2=%f sqd=%f for line in %s frame'
                                   ': target=(%f, %f, %f) prior=(%f, %f, %f) pv=(%f,%f) qv=(%f,%f) '
                                   'a=%f b=%f c=%f discrim=%f '
                                   % (self.name, t1, t2, sqd, local_target.header.frame_id,
                                      local_target.point.x, local_target.point.y, local_target.point.z,
                                      local_prior.point.x, local_prior.point.y, local_prior.point.z,
                                      pv.x, pv.y, qv.x, qv.y, a, b, c, discrim))
                    self._return = 'failed'
                    return None
            elif (t1 > 1.0):
                # all intersections are past the segment
                Logger.loginfo(' %s  Circle is past segment - done! t1=%f t2=%f sqd=%f for line in %s frame: '
                               'target=(%f, %f, %f) prior=(%f, %f, %f) pv=(%f,%f) qv=(%f,%f) '
                               'a=%f b=%f c=%f discrim=%f '
                               % (self.name, t1, t2, sqd, local_target.header.frame_id,
                                  local_target.point.x, local_target.point.y, local_target.point.z,
                                  local_prior.point.x, local_prior.point.y, local_prior.point.z,
                                  pv.x, pv.y, qv.x, qv.y, a, b, c, discrim))
                self._return = 'continue'
                return None
            elif t1 < 0.0 and t2 > 1.0:
                # Segment is contained inside the lookahead circle
                Logger.loginfo(' %s  Circle contains segment - move along! t1=%f t2=%f sqd=%f for line '
                               'in %s frame: target=(%f, %f, %f) prior=(%f, %f, %f) '
                               'pv=(%f,%f) qv=(%f,%f) a=%f b=%f c=%f discrim=%f '
                               % (self.name, t1, t2, sqd, local_target.header.frame_id,
                                  local_target.point.x, local_target.point.y, local_target.point.z,
                                  local_prior.point.x, local_prior.point.y, local_prior.point.z,
                                  pv.x, pv.y, qv.x, qv.y, a, b, c, discrim))
                self._return = 'continue'
                return None
            elif t2 > 1.0:
                # The lookahead circle extends beyond the target point - we are finished here
                Logger.loginfo(' %s  Circle extends past segment for line in %s frame: - done! '
                               't1=%f t2=%f sqd=%f target=(%f, %f, %f) prior=(%f, %f, %f) '
                               'pv=(%f,%f) qv=(%f,%f) a=%f b=%f c=%f discrim=%f '
                               % (self.name, t1, t2, sqd, local_target.header.frame_id,
                                  local_target.point.x, local_target.point.y, local_target.point.z,
                                  local_prior.point.x, local_prior.point.y, local_prior.point.z,
                                  pv.x, pv.y, qv.x, qv.y, a, b, c, discrim))
                self._return = 'continue'
                return None
            else:
                # This is the normal case
                # lookahead circle intersects in the line segment
                # Define the control lookahead point in the robot's frame
                control = deepcopy(local_prior)
                control.point.x = control.point.x + t2 * pv.x  # Control point in the robot frame
                control.point.y = control.point.y + t2 * pv.y  # Control point in the robot frame
                control.point.z = control.point.z + t2 * pv.z  # Control point in the robot frame

                if self._marker_topic:
                    # Update the markers
                    self._reference_marker.header = deepcopy(control.header)
                    self._reference_marker.pose.position = deepcopy(control.point)
                    self._local_target_marker.header = deepcopy(local_target.header)
                    self._local_target_marker.pose.position = deepcopy(local_target.point)

                curvature = 2.0 * control.point.y / (self._lookahead * self._lookahead)
                self._twist.angular.z  = curvature * self._desired_velocity
                return control

        return None

    @staticmethod
    def _make_transform(posn, quat):
        """
        Return 4D homogeneous transform matrix given quaternion.

        Given translation and orientation in normalized quaternion,
        return a 4D homogeneous transform matrix
        https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/index.htm

        Make use of Python duck typing to work with several message forms
        """
        t = np.zeros((4, 4))

        # Rotation matrix (by column)
        t[0][0] = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        t[1][0] = 2.0 * (quat.x * quat.y + quat.z * quat.w)
        t[2][0] = 2.0 * (quat.x * quat.z - quat.y * quat.w)

        t[0][1] = 2.0 * (quat.x * quat.y - quat.z * quat.w)
        t[1][1] = 1 - 2 * (quat.x * quat.x + quat.z * quat.z)
        t[2][1] = 2.0 * (quat.y * quat.z + quat.x * quat.w)

        t[0][2] = 2.0 * (quat.x * quat.z + quat.y * quat.w)
        t[1][2] = 2.0 * (quat.y * quat.z - quat.x * quat.w)
        t[2][2] = 1 - 2 * (quat.x * quat.x + quat.y * quat.y)

        # Translation
        t[0][3] = posn.x
        t[1][3] = posn.y
        t[2][3] = posn.z
        t[3][3] = 1.0

        return t
