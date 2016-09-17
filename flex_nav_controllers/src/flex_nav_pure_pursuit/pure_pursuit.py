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

from copy import deepcopy
from geometry_msgs.msg import TwistStamped, Point, PointStamped, Vector3
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
import math
import numpy as np
import rospy
import tf

class PurePursuit(object):
    """
    Pure Pursuit base class

    Args:
        name (str): The name of the node
    """

    def __init__(self, name):
        self._name = name

        rospy.loginfo('[%s] Pure Pursuit Node loading...', self._name)

        # Load parameters
        self._desired_velocity = rospy.get_param('desired_velocity', 0.2)
        self._max_rotation_rate = rospy.get_param('max_rotation_rate', 10.0)
        self._target_frame = rospy.get_param('target_frame', 'map')
        self._odom_frame = rospy.get_param('odom_frame', 'odom')
        self._robot_frame = rospy.get_param('robot_frame', 'base_footprint')
        self._lookahead_distance = rospy.get_param('lookahead_distance', 0.25)
        self._timeout = rospy.get_param('timeout', 0.08)
        self._cmd_topic = rospy.get_param('cmd_topic', '/create_node/cmd_vel')
        self._odom_topic = rospy.get_param('odom_topic', '/create_node/odom')
        self._marker_topic = rospy.get_param('marker_topic', '/marker')
        self._marker_size = rospy.get_param('marker_size', 0.05)
        self._controller_rate = rospy.get_param('controller_frequency', 20.0)

        # Configure variables
        self._done = False
        self._failed = False
        self._timeout = rospy.Duration(self._timeout)
        self._start_time = None
        self._last_odom = None
        self._last_odom_msg = None
        self._running = False
        self._is_new_goal = False
        self._twist = TwistStamped()
        self._twist.twist.linear.x = self._desired_velocity
        self._twist.twist.angular.z = 0.0
        self._tf_listener = tf.TransformListener()

        # Configre points
        self._target = PointStamped()
        self._target.header.stamp = rospy.Time.now()
        self._target.header.frame_id = self._target_frame

        self._prior = PointStamped()
        self._prior.header.stamp = rospy.Time.now()
        self._prior.header.frame_id = self._target_frame

        self._location = PointStamped()

        # Action server
        self._action_name = name

        # Subscriber
        self._sub = None

        # Odometry subscriber
        self._odom_sub = rospy.Subscriber(self._odom_topic, Odometry, self.odom_cb)

        # Command publisher
        self._cmd_pub = rospy.Publisher(self._cmd_topic, TwistStamped, queue_size = 10)

        # Marker publisher
        self._marker_pub = rospy.Publisher(self._marker_topic, Marker, queue_size = 10)
        self._marker = Marker()
        self._marker.header.frame_id = self._target_frame
        self._marker.header.stamp = rospy.Time.now()
        self._marker.ns = 'pure_pursuit_waypoints'
        self._marker.type = Marker.SPHERE
        self._marker.action = Marker.ADD
        self._marker.scale.x = self._marker_size
        self._marker.scale.y = self._marker_size
        self._marker.scale.z = self._marker_size
        self._marker.color.a = 1.0
        self._marker.color.r = 0.0
        self._marker.color.g = 0.0
        self._marker.color.b = 1.0

        rospy.loginfo('[%s] Pure Pursuit Node loaded!', self._name)

    def execute(self, goal):
        """
        The call back for the ActionServer

        Args:
            goal (ActionGoal): The goal to process
        """

        pass

    def set_succeeded(self, result):
        """
        Sets a goal as succeeded and cleans everything up

        Args:
            result (ActionResult): The result to be published
        """

        ts = TwistStamped()
        ts.header.stamp = rospy.Time.now()
        self._cmd_pub.publish(ts)
        self._server.set_succeeded(result)
        self._marker.color.a = 1.0
        self._marker.color.r = 0.0
        self._marker.color.g = 0.0
        self._marker.color.b = 1.0 # Blue on success
        self._marker_pub.publish(self._marker)
        self._running = False
        self._target = PointStamped()
        self._prior = PointStamped()

    def set_aborted(self, result):
        """
        Sets a goal as aborted and cleans everything up

        Args:
            result (ActionResult): The result to be published
        """

        ts = TwistStamped()
        ts.header.stamp = rospy.Time.now()
        self._cmd_pub.publish(ts)
        self._server.set_aborted(result)
        self._marker.color.a = 1.0
        self._marker.color.r = 0.8 # Red on failure
        self._marker.color.g = 0.0
        self._marker.color.b = 0.0
        self._marker_pub.publish(self._marker)
        self._running = False
        self._target = PointStamped()
        self._prior = PointStamped()

    def set_preempted(self, result):
        """
        Sets a goal as preempted and cleans everything up

        Args:
            result (ActionResult): The result to be published
        """

        ts = TwistStamped()
        ts.header.stamp = rospy.Time.now()
        self._cmd_pub.publish(ts)
        self._server.set_preempted(result)
        self._marker.color.a = 1.0
        self._marker.color.r = 1.0
        self._marker.color.g = 0.6
        self._marker.color.b = 0.0
        self._marker_pub.publish(self._marker)
        self._running = False
        self._target = PointStamped()
        self._prior = PointStamped()

    def get_current_target(self, poses):
        """
        Locates the goal point to navigate to from a list of poses

        Args:
            poses (Pose[]): An array of poses
        """

        dr = 0.0
        while (dr < self._lookahead_distance and self._running and not rospy.is_shutdown()):
            if self._indice < len(poses):
                # Get target given valid index
                self._target.point = poses[self._indice].pose.position

                dr = np.sqrt((self._target.point.x - self._location.point.x)**2 + (self._target.point.y - self._location.point.y)**2)
                if dr >= self._lookahead_distance:
                    # Found target across lookahead boundary
                    self._prior.point = poses[self._indice - 1].pose.position
                    dr0 = np.sqrt((self._prior.point.x - self._location.point.x)**2 + (self._prior.point.y - self._location.point.y)**2)
                    if (dr0 <= self._lookahead_distance):
                        rospy.logdebug(
                            '[%s] Accepting index = %d of %d  target=(%f,%f) prior=(%f, %f) location=(%f,%f) dr=%f dr0=%f lookahead=%f',
                            self._name, self._indice, len(poses), self._target.point.x, self._target.point.y,
                            self._prior.point.x, self._prior.point.y,
                            self._location.point.x, self._location.point.y, dr, dr0, self._lookahead_distance
                        )
                        return True
                    else:
                        rospy.logdebug(
                            '[%s] Rejected index = %d of %d  target=(%f,%f) prior=(%f, %f) location=(%f,%f) dr=%f because dr0=%f > lookahead=%f',
                            self._name, self._indice, len(poses), self._target.point.x, self._target.point.y,
                            self._prior.point.x, self._prior.point.y,
                            self._location.point.x, self._location.point.y, dr, dr0, self._lookahead_distance
                        )
                        return False
            else:
                # Failed to find valid index
                rospy.logdebug(
                    '[%s] Failed to find index =%d >= %d target=(%f,%f) prior=(%f, %f) location=(%f,%f) dr=%f lookahead=%f',self._indice, len(poses),
                    self._name, self._target.point.x, self._target.point.y, self._prior.point.x, self._prior.point.y,
                    self._location.point.x, self._location.point.y, dr, self._lookahead_distance
                )
                return False

            self._indice += 1 # Check next index

        rospy.logerr('[%s] Invalid get_current_error - how did we get here?', self._name)
        self._failed = True
        return False


    def calculateLineTwist(self, local_prior, local_target):
        """
        Calculates the Twist needed to reach the target point

        Args:
            local_prior (PoseStamped): The previous point before the target
            local_target (PoseStamped): The target point to navigate towards
        """

        # Define a line segment from prior to the target (assume 2D)
        pv = Vector3(local_target.point.x - local_prior.point.x, local_target.point.y - local_prior.point.y, 0.0)
        qv = Vector3(local_prior.point.x - self._location.point.x, local_prior.point.y - self._location.point.y, 0.0)

        # Find intersection of line segment with lookahead circle centered at the  robot
        a = pv.x * pv.x + pv.y * pv.y
        b = 2.0 * (qv.x * pv.x + qv.y * pv.y)
        c = (qv.x * qv.x + qv.y * qv.y) - self._lookahead_distance * self._lookahead_distance

        if a < 1.e-9:
            rospy.logerr(
                '[%s] Invalid prior and target for line: target=(%f, %f, %f) prior=(%f, %f, %f) robot=(%f,%f, %f)  pv=(%f,%f) qv=(%f,%f) a=%f b=%f c=%f',
                self._name, local_target.point.x, local_target.point.y, local_target.point.z, local_prior.point.x, local_prior.point.y, local_prior.point.z,
                self._location.point.x, self._location.point.y, self._location.point.z,
                pv.x, pv.y, qv.x, qv.y, a, b, c
            )
            self._failed = True
            return None

        discrim = b * b - 4 * a * c

        if discrim < 0.0:
             # No intersection - this should not be unless bad start or localization perturbation
            rospy.logerr(
                '[%s] No path recovery - no intersection for line: target=(%f, %f, %f) prior=(%f, %f, %f) robot=(%f, %f, %f)  pv=(%f, %f) qv=(%f, %f) a=%f b=%f c=%f discrim=%f',
                self._name, local_target.point.x, local_target.point.y, local_target.point.z, local_prior.point.x, local_prior.point.y, local_prior.point.z,
                self._location.point.x, self._location.point.y, self._location.point.z,
                pv.x, pv.y, qv.x, qv.y, a, b, c, discrim
            )
            self._failed = True
            return None
        else:
            # solve quadratic equation for intersection points
            sqd = math.sqrt(discrim)
            t1 = (-b - sqd) / (2 * a)  # min value
            t2 = (-b + sqd) / (2 * a)  # max value

            if t2 < t1:
                rospy.logerr(
                    '[%s] Say what!: t1=%f t2=%f sqd=%f target=(%f, %f, %f) prior=(%f, %f, %f) robot=(%f, %f, %f)  pv=(%f, %f) qv=(%f, %f) a=%f b=%f c=%f discrim=%f',
                    self._name, t1, t2, sqd, local_target.point.x, local_target.point.y, local_target.point.z, local_prior.point.x, local_prior.point.y, local_prior.point.z,
                    self._location.point.x, self._location.point.y, self._location.point.z,
                    pv.x, pv.y, qv.x, qv.y, a, b, c, discrim
                )
                self._failed = True
                return None

            if t2 < 0.0:
                # all intersections are behind the segment - this should not be unless bad start or localization perturbation
                if t2 > -0.1:
                    # likely due to localization perturbation
                    rospy.logerr('[%s] Circle is before segment!', self._name)
                    rospy.logdebug(
                        '[%s] Circle is before segment - continue prior motion!: t1=%f t2=%f sqd=%f target=(%f, %f, %f) prior=(%f, %f, %f) robot=(%f, %f, %f)  pv=(%f, %f) qv=(%f, %f) a=%f b=%f c=%f discrim=%f',
                        self._name, t1, t2, sqd, local_target.point.x, local_target.point.y, local_target.point.z, local_prior.point.x, local_prior.point.y, local_prior.point.z,
                        self._location.point.x, self._location.point.y, self._location.point.z,
                        pv.x, pv.y, qv.x, qv.y, a, b, c, discrim
                    )
                    return None
                else:
                    rospy.logerr('[%s] Circle is before segment!', self._name)
                    rospy.logdebug(
                        '[%s] Circle is before segment!: t1=%f t2=%f sqd=%f target=(%f, %f, %f) prior=(%f, %f, %f) robot=(%f, %f, %f)  pv=(%f, %f) qv=(%f, %f) a=%f b=%f c=%f discrim=%f',
                        self._name, t1, t2, sqd, local_target.point.x, local_target.point.y, local_target.point.z, local_prior.point.x, local_prior.point.y, local_prior.point.z,
                        self._location.point.x, self._location.point.y, self._location.point.z,
                        pv.x, pv.y, qv.x, qv.y, a, b, c, discrim
                    )
                    self._failed = True
                    return None
            elif t1 > 1.0:
                # all intersections are past the segment
                rospy.logerr('[%s] Circle is past segment', self._name)
                rospy.logdebug(
                    '[%s] Circle is past segment - done!: t1=%f t2=%f sqd=%f target=(%f, %f, %f) prior=(%f, %f, %f) robot=(%f, %f, %f)  pv=(%f, %f) qv=(%f, %f) a=%f b=%f c=%f discrim=%f',
                    self._name, t1, t2, sqd, local_target.point.x, local_target.point.y, local_target.point.z, local_prior.point.x, local_prior.point.y, local_prior.point.z,
                    self._location.point.x, self._location.point.y, self._location.point.z,
                    pv.x, pv.y, qv.x, qv.y, a, b, c, discrim
                )
                self._done = True
                return None
            elif t1 < 0.0 and t2 > 1.0:
                # Segment is contained inside the lookahead circle
                rospy.logerr('[%s] Circle contains segment', self._name)
                rospy.logdebug(
                    '[%s] Circle contains segment - move along!: t1=%f t2=%f sqd=%f target=(%f, %f, %f) prior=(%f, %f, %f) robot=(%f, %f, %f)  pv=(%f, %f) qv=(%f, %f) a=%f b=%f c=%f discrim=%f',
                    self._name, t1, t2, sqd, local_target.point.x, local_target.point.y, local_target.point.z, local_prior.point.x, local_prior.point.y, local_prior.point.z,
                    self._location.point.x, self._location.point.y, self._location.point.z,
                    pv.x, pv.y, qv.x, qv.y, a, b, c, discrim
                )
                self._done = True
                return None
            elif t2 > 1.0:
                # The lookahead circle extends beyond the target point - we are finished here
                rospy.logerr('[%s] Circle extends past segment', self._name)
                rospy.logdebug(
                    '[%s] Circle extends past segment - done!: t1=%f t2=%f sqd=%f target=(%f, %f, %f) prior=(%f, %f, %f) robot=(%f, %f, %f)  pv=(%f, %f) qv=(%f, %f) a=%f b=%f c=%f discrim=%f',
                    self._name, t1, t2, sqd, local_target.point.x, local_target.point.y, local_target.point.z, local_prior.point.x, local_prior.point.y, local_prior.point.z,
                    self._location.point.x, self._location.point.y, self._location.point.z,
                    pv.x, pv.y, qv.x, qv.y, a, b, c, discrim
                )
                self._done = True
                return None
            else:
                # This is the normal case
                # Must be line segment
                control = deepcopy(local_prior)
                control.header.stamp = self._tf_listener.getLatestCommonTime(local_prior.header.frame_id, self._robot_frame)
                control.point.x = control.point.x + t2 * pv.x # Control point in the odometry frame
                control.point.y = control.point.y + t2 * pv.y # Control point in the odometry frame
                control.point.z = control.point.z + t2 * pv.z # Control point in the odometry frame

                self._marker.header = self._location.header
                self._marker.pose.position = control.point

                try:
                    control_robot = self._tf_listener.transformPoint(self._robot_frame, control)
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                    rospy.logwarn('[%s] Failed to get the transformBody:\n%s', self._name, str(e))
                    self._failed = True
                    return None
                except Exception as e:
                    rospy.logwarn('[%s] Failed to get the transformBody due to unknown error:\n%s', self._name, str(e))
                    self._failed = True
                    return None

                if not control_robot:
                    return None

                curvature = 2.0 * control_robot.point.y / (self._lookahead_distance * self._lookahead_distance)
                self._twist.twist.angular.z = curvature * self._desired_velocity

                return control_robot

        return None

    def transformFrame(self, pose, target_frame_id):
        """
        Transforms a PoseStamped into another map frame

        Args:
            pose (PoseStamped): The pose to transform
            target_frame_id (str): The frame to transform to
        """

        odom_position = PointStamped()
        odom_position.header = pose.header
        odom_position.point = pose.pose.pose.position

        try:
            # self._tf_listener.waitForTransform(self._target_frame, odometry.header.frame_id, odometry.header.stamp, self._timeout)
            return self._tf_listener.transformPoint(target_frame_id, odom_position)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn('[%s] Failed to get the transformFrame to target_frame:\n%s', self._name, str(e))
            self._failed = True
            return None
        except Exception as e:
            rospy.logwarn('[%s] Failed to get the transformFrame to target frame due to unknown error:\n%s', self._name, str(e))
            self._failed = True
            return None

    def odom_cb(self, data):
        """
        The call back for the odometry subscriber

        Args:
            data (Odometry): The odometry message to process
        """

        self._last_odom_msg = data
