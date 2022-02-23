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
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.exceptions import ParameterNotDeclaredException
from geometry_msgs.msg import TwistStamped, Point, PointStamped, Vector3
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
import tf2_ros


class PurePursuit(object):
    """
    Pure Pursuit base class

    Args:
        node (rclpy.Node): The node using PurePursuit
    """

    def __init__(self, node):
        self.node = node
        self.node.get_logger().info('{} Pure Pursuit Node loading...'.format(self.get_name()))

        # Load parameters
        try:
            self._desired_velocity = self.node.get_parameter('desired_velocity')
        except ParameterNotDeclaredException as e:
            self.node.declare_parameter('desired_velocity', 0.2)
            self._desired_velocity = self.node.get_parameter('desired_velocity')

        try:
            self._max_rotation_rate = self.node.get_parameter('max_rotation_rate')
        except ParameterNotDeclaredException as e:
            self.node.declare_parameter('max_rotation_rate', 10.0)
            self._max_rotation_rate = self.node.get_parameter('max_rotation_rate')

        try:
            self._target_frame = self.node.get_parameter('target_frame')
        except ParameterNotDeclaredException as e:
            self.node.declare_parameter('target_frame', 'map')
            self._target_frame = self.node.get_parameter('target_frame')

        try:
            self._odom_frame = self.node.get_parameter('odom_frame')
        except ParameterNotDeclaredException as e:
            self.node.declare_parameter('odom_frame', 'odom')
            self._odom_frame = self.node._nget_parameter('odom_frame')

        try:
            self._robot_frame = self.node.get_parameter('robot_frame')
        except ParameterNotDeclaredException as e:
            self.node.declare_parameter('robot_frame', 'base_footprint')
            self._robot_frame = self.node.get_parameter('robot_frame')

        try:
            self._lookahead_distance = self.node.get_parameter('lookahead_distance')
        except ParameterNotDeclaredException as e:
            self.node.declare_parameter('lookahead_distance', 0.25)
            self._lookahead_distance = self.node.get_parameter('lookahead_distance')

        try:
            self._timeout = self.node.get_parameter('timeout')
        except ParameterNotDeclaredException as e:
            self.node.declare_parameter('timeout', 0.08)
            self._timeout = self.node.get_parameter('timeout')

        try:
            self._cmd_topic = self.get_parameter('cmd_topic')
        except ParameterNotDeclaredException as e:
            self.declare_parameter('cmd_topic', '/createnode/cmd_vel')
            self._cmd_topic = self.get_parameter('cmd_topic')

        try:
            self._odom_topic = self.node.get_parameter('odom_topic')
        except ParameterNotDeclaredException as e:
            self.node.declare_parameter('odom_topic', '/createnode/odom')
            self._odom_topic = self.node.get_parameter('odom_topic')

        try:
            self._marker_topic = self.node.get_parameter('marker_topic')
        except ParameterNotDeclaredException as e:
            self.node.declare_parameter('marker_topic', '/marker')
            self._marker_topic = self.node.get_parameter('marker_topic')

        try:
            self._marker_size = self.node.get_parameter('marker_size')
        except ParameterNotDeclaredException as e:
            self.node.declare_parameter('marker_size', 0.05)
            self._marker_size = self.node.get_parameter('marker_size')

        try:
            self._controller_rate = self.node.get_parameter('controller_frequency')
        except ParameterNotDeclaredException as e:
            self.node.declare_parameter('controller_frequency', 20.0)
            self._controller_rate = self.node.get_parameter('controller_frequency')

        # Configure variables
        self._done = False
        self._failed = False
        self._timeout = Duration(seconds=(self._timeout.value))
        self._start_time = None
        self._last_odom = None
        self._last_odom_msg = None
        self._running = False
        self._is_new_goal = False
        self._twist = TwistStamped()
        self._twist.twist.linear.x = self._desired_velocity.value
        self._twist.twist.angular.z = 0.0
        self._tf_buffer = tf2_ros.Buffer(self._timeout)
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Configre points
        self._target = PointStamped()
        self._target.header.stamp = self.get_clock().now().to_msg()
        self._target.header.frame_id = self._target_frame.value

        self._prior = PointStamped()
        # self._prior.header.stamp = rospy.Time.now()
        self._prior.header.stamp = self.get_clock().now().to_msg()
        self._prior.header.frame_id = self._target_frame.value

        self._location = PointStamped()

        # Action server
        self._action_name = name

        # Subscriber
        self._sub = None

        # Odometry subscriber
        self._odom_sub = self.create_subscription(Odometry, self._odom_topic.value, self.odom_cb, 10)

        # Command publisher
        self._cmd_pub = self.create_publisher(TwistStamped, self._cmd_topic.value, 10)

        # Marker publisher
        self._marker_pub = self.create_publisher(Marker, self._marker_topic.value, 10)
        self._marker = Marker()
        self._marker.header.frame_id = self._target_frame.value
        self._marker.header.stamp = self.get_clock().now().to_msg()
        self._marker.ns = 'pure_pursuit_waypoints'
        self._marker.type = Marker.SPHERE
        self._marker.action = Marker.ADD
        self._marker.scale.x = self._marker_size.value
        self._marker.scale.y = self._marker_size.value
        self._marker.scale.z = self._marker_size.value
        self._marker.color.a = 1.0
        self._marker.color.r = 0.0
        self._marker.color.g = 0.0
        self._marker.color.b = 1.0

        self.node.get_logger().info('{} Pure Pursuit Node loaded!'.format(self.get_name()))

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
        ts.header.stamp = self.get_clock().now().to_msg()
        self._cmd_pub.publish(ts)
        # self._server_goal.set_succeeded(result)
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
        ts.header.stamp = self.get_clock().now().to_msg()
        self._cmd_pub.publish(ts)
        # self._server_goal.set_aborted(result)
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
        ts.header.stamp = self.get_clock().now().to_msg()
        self._cmd_pub.publish(ts)
        # self._server_goal.set_preempted(result)
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
        while (dr < self._lookahead_distance.value and self._running and rclpy.ok()):
            if self._indice < len(poses):
                # Get target given valid index
                self._target.point = poses[self._indice].pose.position

                dr = np.sqrt((self._target.point.x - self._location.point.x)**2 + (self._target.point.y - self._location.point.y)**2)
                if dr >= self._lookahead_distance.value:
                    # Found target across lookahead boundary
                    self._prior.point = poses[self._indice - 1].pose.position
                    dr0 = np.sqrt((self._prior.point.x - self._location.point.x)**2 + (self._prior.point.y - self._location.point.y)**2)
                    if (dr0 <= self._lookahead_distance.value):
                        self.node.get_logger().debug(
                            '%s Accepting index = %d of %d  target=(%f,%f) prior=(%f, %f) location=(%f,%f) dr=%f dr0=%f lookahead=%f'
                            % (self.get_name(), self._indice, len(poses), self._target.point.x, self._target.point.y,
                            self._prior.point.x, self._prior.point.y,
                            self._location.point.x, self._location.point.y, dr, dr0, self._lookahead_distance.value)
                        )

                        return True
                    else:
                        self.node.get_logger().debug(
                            '%s Rejected index = %d of %d  target=(%f,%f) prior=(%f, %f) location=(%f,%f) dr=%f because dr0=%f > lookahead=%f'
                            % (self.get_name(), self._indice, len(poses), self._target.point.x, self._target.point.y,
                            self._prior.point.x, self._prior.point.y,
                            self._location.point.x, self._location.point.y, dr, dr0, self._lookahead_distance.value)
                        )

                        return False
            else:
                # Failed to find valid index
                self.node.get_logger().debug(
                    '%s Failed to find index =%d >= %d target=(%f,%f) prior=(%f, %f) location=(%f,%f) dr=%f lookahead=%f',self._indice, len(poses)
                    % (self.get_name(), self._target.point.x, self._target.point.y, self._prior.point.x, self._prior.point.y,
                    self._location.point.x, self._location.point.y, dr, self._lookahead_distance.value)
                )

                return False

            self._indice += 1 # Check next index

        self.node.get_logger().error('{} Invalid get_current_error - how did we get here?'.format(self.get_name()))
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
        c = (qv.x * qv.x + qv.y * qv.y) - self._lookahead_distance.value * self._lookahead_distance.value

        if a < 1.e-9:
            self.node.get_logger().error(
                '%s Invalid prior and target for line: target=(%f, %f, %f) prior=(%f, %f, %f) robot=(%f,%f, %f)  pv=(%f,%f) qv=(%f,%f) a=%f b=%f c=%f'
                % (self.get_name(), local_target.point.x, local_target.point.y, local_target.point.z, local_prior.point.x, local_prior.point.y, local_prior.point.z,
                self._location.point.x, self._location.point.y, self._location.point.z,
                pv.x, pv.y, qv.x, qv.y, a, b, c)
            )

            self._failed = True
            return None

        discrim = b * b - 4 * a * c

        if discrim < 0.0:
             # No intersection - this should not be unless bad start or localization perturbation
            self.node.get_logger().error(
                '%s No path recovery - no intersection for line: target=(%f, %f, %f) prior=(%f, %f, %f) robot=(%f, %f, %f)  pv=(%f, %f) qv=(%f, %f) a=%f b=%f c=%f discrim=%f'
                % (self.get_name(), local_target.point.x, local_target.point.y, local_target.point.z, local_prior.point.x, local_prior.point.y, local_prior.point.z,
                self._location.point.x, self._location.point.y, self._location.point.z,
                pv.x, pv.y, qv.x, qv.y, a, b, c, discrim)
            )

            self._failed = True
            return None
        else:
            # solve quadratic equation for intersection points
            sqd = math.sqrt(discrim)
            t1 = (-b - sqd) / (2 * a)  # min value
            t2 = (-b + sqd) / (2 * a)  # max value

            if t2 < t1:
                self.node.get_logger().error(
                    '%s Say what!: t1=%f t2=%f sqd=%f target=(%f, %f, %f) prior=(%f, %f, %f) robot=(%f, %f, %f)  pv=(%f, %f) qv=(%f, %f) a=%f b=%f c=%f discrim=%f'
                    % (self.get_name(), t1, t2, sqd, local_target.point.x, local_target.point.y, local_target.point.z, local_prior.point.x, local_prior.point.y, local_prior.point.z,
                    self._location.point.x, self._location.point.y, self._location.point.z,
                    pv.x, pv.y, qv.x, qv.y, a, b, c, discrim)
                )

                self._failed = True
                return None

            if t2 < 0.0:
                # all intersections are behind the segment - this should not be unless bad start or localization perturbation
                if t2 > -0.1:
                    # likely due to localization perturbation
                    self.node.get_logger().error('{} Circle is before segment!'.format(self.get_name()))
                    self.node.get_logger().debug(
                        '%s Circle is before segment - continue prior motion!: t1=%f t2=%f sqd=%f target=(%f, %f, %f) prior=(%f, %f, %f) robot=(%f, %f, %f)  pv=(%f, %f) qv=(%f, %f) a=%f b=%f c=%f discrim=%f'
                        % (self.get_name(), t1, t2, sqd, local_target.point.x, local_target.point.y, local_target.point.z, local_prior.point.x, local_prior.point.y, local_prior.point.z,
                        self._location.point.x, self._location.point.y, self._location.point.z,
                        pv.x, pv.y, qv.x, qv.y, a, b, c, discrim)
                    )

                    return None
                else:
                    self.node.get_logger().error('{} Circle is before segment!'.format(self.get_name()))
                    self.node.get_logger().debug(
                        '%s Circle is before segment!: t1=%f t2=%f sqd=%f target=(%f, %f, %f) prior=(%f, %f, %f) robot=(%f, %f, %f)  pv=(%f, %f) qv=(%f, %f) a=%f b=%f c=%f discrim=%f'
                        % (self.get_name(), t1, t2, sqd, local_target.point.x, local_target.point.y, local_target.point.z, local_prior.point.x, local_prior.point.y, local_prior.point.z,
                        self._location.point.x, self._location.point.y, self._location.point.z,
                        pv.x, pv.y, qv.x, qv.y, a, b, c, discrim)
                    )

                    self._failed = True
                    return None
            elif t1 > 1.0:
                # all intersections are past the segment
                self.node.get_logger().error('{} Circle is past segment'.format(self.get_name()))
                self.node.get_logger().debug(
                    '%s Circle is past segment - done!: t1=%f t2=%f sqd=%f target=(%f, %f, %f) prior=(%f, %f, %f) robot=(%f, %f, %f)  pv=(%f, %f) qv=(%f, %f) a=%f b=%f c=%f discrim=%f'
                    % (self.get_name(), t1, t2, sqd, local_target.point.x, local_target.point.y, local_target.point.z, local_prior.point.x, local_prior.point.y, local_prior.point.z,
                    self._location.point.x, self._location.point.y, self._location.point.z,
                    pv.x, pv.y, qv.x, qv.y, a, b, c, discrim)
                )

                self._done = True
                return None
            elif t1 < 0.0 and t2 > 1.0:
                # Segment is contained inside the lookahead circle
                self.node.get_logger().error('{} Circle contains segment'.format(self.get_name()))
                self.node.get_logger().debug(
                    '%s Circle contains segment - move along!: t1=%f t2=%f sqd=%f target=(%f, %f, %f) prior=(%f, %f, %f) robot=(%f, %f, %f)  pv=(%f, %f) qv=(%f, %f) a=%f b=%f c=%f discrim=%f'
                    % (self.get_name(), t1, t2, sqd, local_target.point.x, local_target.point.y, local_target.point.z, local_prior.point.x, local_prior.point.y, local_prior.point.z,
                    self._location.point.x, self._location.point.y, self._location.point.z,
                    pv.x, pv.y, qv.x, qv.y, a, b, c, discrim)
                )

                self._done = True
                return None
            elif t2 > 1.0:
                # The lookahead circle extends beyond the target point - we are finished here
                self.node.get_logger().error('{} Circle extends past segment'.format(self.get_name()))
                self.node.get_logger().debug(
                    '%s Circle extends past segment - done!: t1=%f t2=%f sqd=%f target=(%f, %f, %f) prior=(%f, %f, %f) robot=(%f, %f, %f)  pv=(%f, %f) qv=(%f, %f) a=%f b=%f c=%f discrim=%f'
                    % (self.get_name(), t1, t2, sqd, local_target.point.x, local_target.point.y, local_target.point.z, local_prior.point.x, local_prior.point.y, local_prior.point.z,
                    self._location.point.x, self._location.point.y, self._location.point.z,
                    pv.x, pv.y, qv.x, qv.y, a, b, c, discrim)
                )

                self._done = True
                return None
            else:
                # This is the normal case
                # Must be line segment
                control = deepcopy(local_prior)
                control.header.stamp = self._tf_listener.getLatestCommonTime(local_prior.header.frame_id, self._robot_frame.value)
                control.point.x = control.point.x + t2 * pv.x # Control point in the odometry frame
                control.point.y = control.point.y + t2 * pv.y # Control point in the odometry frame
                control.point.z = control.point.z + t2 * pv.z # Control point in the odometry frame

                self._marker.header = self._location.header
                self._marker.pose.position = control.point

                try:
                    control_robot = self._tf_listener.transformPoint(self._robot_frame.value, control)
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                    self.node.get_logger().warn('%s Failed to get the transformBody:\n%s' % (self.get_name(), str(e)))
                    self._failed = True
                    return None
                except Exception as e:
                    self.node.get_logger().warn('%s Failed to get the transformBody due to unknown error:\n%s' % (self.get_name(), str(e)))
                    self._failed = True
                    return None

                if not control_robot:
                    return None

                curvature = 2.0 * control_robot.point.y / (self._lookahead_distance.value * self._lookahead_distance.value)
                self._twist.twist.angular.z = curvature * self._desired_velocity.value

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
            return self._tf_listener.transformPoint(target_frame_id, odom_position)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.node.get_logger().warn('%s Failed to get the transformFrame to target_frame:\n%s' % (self.get_name(), str(e)))
            self._failed = True
            return None
        except Exception as e:
            self.node.get_logger().logwarn('%s Failed to get the transformFrame to target frame due to unknown error:\n%s' % (self.get_name(), str(e)))
            self._failed = True
            return None

    def odom_cb(self, data):
        """
        The call back for the odometry subscriber

        Args:
            data (Odometry): The odometry message to process
        """

        self._last_odom_msg = data
