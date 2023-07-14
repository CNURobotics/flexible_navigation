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
#  1. Redistributions of source code must retain the above copyright notice,
#     this list of conditions and the following disclaimer.
#
#  2. Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#
#  3. Neither the name of the copyright holder nor the names of its
#     contributors may be used to endorse or promote products derived from
#     this software without specific prior written permission.
#
#     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#     "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#     LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#     FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#     COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#     INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#     BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#     CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#     LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
#     WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#     POSSIBILITY OF SUCH DAMAGE.
###############################################################################

"""Provides generic implementation of pure pursuit algorithm."""

from copy import deepcopy
import math
import traceback
import numpy as np

import rclpy
from rclpy.action import CancelResponse
from rclpy.action import GoalResponse
from rclpy.duration import Duration
from rclpy.exceptions import ParameterNotDeclaredException
from rclpy.node import Node

from geometry_msgs.msg import Point, PointStamped, Pose, PoseStamped, Twist, TwistStamped, Vector3
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros


class PurePursuit(Node):
    """Pure Pursuit base class."""

    def __init__(self, name='pure_pursuit'):
        super().__init__(name)
        self.get_logger().info(f'{self.get_name()} Pure Pursuit Node loading...')

        self._action_server = None  # Set in derived class
        self._result_type = None
        self._feedback_type = None
        self._indice = 0
        self._goal = None

        # Load parameters
        try:
            self._desired_velocity = self.get_parameter('desired_velocity')
        except ParameterNotDeclaredException:
            self.declare_parameter('desired_velocity', 0.2)
            self._desired_velocity = self.get_parameter('desired_velocity')

        try:
            self._max_rotation_rate = self.get_parameter('max_rotation_rate')
        except ParameterNotDeclaredException:
            self.declare_parameter('max_rotation_rate', 10.0)
            self._max_rotation_rate = self.get_parameter('max_rotation_rate')

        try:
            self._target_frame = self.get_parameter('target_frame')
        except ParameterNotDeclaredException:
            self.declare_parameter('target_frame', 'map')
            self._target_frame = self.get_parameter('target_frame')

        # try:
        #     self._odom_frame = self.get_parameter('odom_frame')
        # except ParameterNotDeclaredException:
        #     self.declare_parameter('odom_frame', 'odom')
        #     self._odom_frame = self.get_parameter('odom_frame')

        try:
            self._robot_frame = self.get_parameter('robot_frame')
        except ParameterNotDeclaredException:
            self.declare_parameter('robot_frame', 'base_footprint')
            self._robot_frame = self.get_parameter('robot_frame')

        try:
            self._lookahead_distance = self.get_parameter('lookahead_distance')
        except ParameterNotDeclaredException:
            self.declare_parameter('lookahead_distance', 0.25)
            self._lookahead_distance = self.get_parameter('lookahead_distance')

        try:
            self._timeout = self.get_parameter('timeout')
        except ParameterNotDeclaredException:
            self.declare_parameter('timeout', 0.08)
            self._timeout = self.get_parameter('timeout')

        try:
            self._cmd_topic = self.get_parameter('cmd_topic')
        except ParameterNotDeclaredException:
            self.declare_parameter('cmd_topic', 'cmd_vel')
            self._cmd_topic = self.get_parameter('cmd_topic')

        # Optional, publish as a TwistStamped
        try:
            self._cmd_topic_stamped = self.get_parameter('cmd_topic_stamped')
        except ParameterNotDeclaredException:
            self.declare_parameter('cmd_topic_stamped', '')
            self._cmd_topic_stamped = self.get_parameter('cmd_topic_stamped')

        try:
            self._marker_topic = self.get_parameter('marker_topic')
        except ParameterNotDeclaredException:
            self.declare_parameter('marker_topic', 'pure_pursuit_markers')
            self._marker_topic = self.get_parameter('marker_topic')

        try:
            self._marker_size = self.get_parameter('marker_size')
        except ParameterNotDeclaredException:
            self.declare_parameter('marker_size', 0.05)
            self._marker_size = self.get_parameter('marker_size')

        try:
            self._controller_rate = self.get_parameter('controller_frequency')
        except ParameterNotDeclaredException:
            self.declare_parameter('controller_frequency', 20.0)
            self._controller_rate = self.get_parameter('controller_frequency')

        try:
            self._halt_on_completion = self.get_parameter('halt_on_completion')
        except ParameterNotDeclaredException:
            self.declare_parameter('halt_on_completion', True)
            self._halt_on_completion = self.get_parameter('halt_on_completion')

        try:
            self._halt_on_failure = self.get_parameter('halt_on_failure')
        except ParameterNotDeclaredException:
            self.declare_parameter('halt_on_failure', True)
            self._halt_on_failure = self.get_parameter('halt_on_failure')

        try:
            self._halt_on_cancel = self.get_parameter('halt_on_cancel')
        except ParameterNotDeclaredException:
            self.declare_parameter('halt_on_cancel', True)
            self._halt_on_cancel = self.get_parameter('halt_on_cancel')

        # Configure variables
        self._lookahead_distance_squared = self._lookahead_distance.value**2
        self._done = False
        self._failed = False
        self._timeout = Duration(seconds=(self._timeout.value))
        self._start_time = None
        # self._last_odom = None
        # self._last_odom_msg = None
        self._running = False
        self._is_new_goal = False
        self._twist = Twist()
        self._twist.linear.x = self._desired_velocity.value
        self._twist.angular.z = 0.0
        self._tf_buffer = tf2_ros.Buffer(self._timeout)
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Configure segment points
        self._target = None
        self._prior = None
        self._control = None

        # Action server
        self._action_name = self.get_name()

        # Subscriber
        self._sub = None

        # Command publisher
        if isinstance(self._cmd_topic.value, str) and len(self._cmd_topic.value) != 0:
            self._cmd_pub = self.create_publisher(Twist, self._cmd_topic.value, 10)
        else:
            self._cmd_topic = None
            self._cmd_pub = None

        if isinstance(self._cmd_topic_stamped.value, str) and len(self._cmd_topic_stamped.value) != 0:
            self._twist_stamped = TwistStamped()
            self._twist_stamped.twist = self._twist
            self._cmd_pub_stamped = self.create_publisher(TwistStamped, self._cmd_topic_stamped.value, 10)
        else:
            self._twist_stamped = None
            self._cmd_topic_stamped = None
            self._cmd_pub_stamped = None

        assert self._cmd_pub or self._cmd_pub_stamped, "Must define at least cmd publishing topic"

        # Marker publisher
        if isinstance(self._marker_topic.value, str) and len(self._marker_topic.value) > 0:
            self._marker_pub = self.create_publisher(MarkerArray, self._marker_topic.value, 10)
            # Marker point that we are steering toward along the intersection
            self._reference_marker = Marker()
            self._reference_marker.header.frame_id = self._target_frame.value
            self._reference_marker.header.stamp = self.get_clock().now().to_msg()
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
            self._reference_marker.scale.x = self._marker_size.value * 0.75
            self._reference_marker.scale.y = self._marker_size.value * 0.75
            self._reference_marker.scale.z = self._marker_size.value * 0.75
            self._reference_marker.color.a = 0.0  # Add, but make invisible at first
            self._reference_marker.color.r = 1.0
            self._reference_marker.color.g = 0.0
            self._reference_marker.color.b = 1.0

            # End point of current segment
            self._local_target_marker = Marker()
            self._local_target_marker.header.frame_id = self._target_frame.value
            self._local_target_marker.header.stamp = self.get_clock().now().to_msg()
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
            self._local_target_marker.scale.x = self._marker_size.value
            self._local_target_marker.scale.y = self._marker_size.value
            self._local_target_marker.scale.z = self._marker_size.value
            self._local_target_marker.color.a = 0.0  # Add, but make invisible at first
            self._local_target_marker.color.r = 1.0
            self._local_target_marker.color.g = 0.0
            self._local_target_marker.color.b = 1.0
            self._marker_array = MarkerArray()
            self._marker_array.markers.append(self._reference_marker)
            self._marker_array.markers.append(self._local_target_marker)

        else:
            self._marker_pub = None
            self._reference_marker = None
            self._local_target_marker = None
            self._marker_array = None

        self.get_logger().info(f"{self.get_name()} Pure Pursuit Node loaded!")

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        # Accepts or rejects a client request to begin an action
        self.get_logger().info('Received goal request')
        self._goal = goal_request
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        # Accepts or rejects a client request to cancel an action
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute(self, goal_handle):  # noqa: W613 # pylint: disable=W0613
        """
        Initialize values as super().execute.

        @param  goal_handle The goal to process
        """
        self._lookahead_distance_squared = self._lookahead_distance.value**2

        if self._marker_pub:
            self.get_logger().info('Activate pure pursuit markers ...')
            self._reference_marker.color.a = 1.0
            self._reference_marker.color.r = 0.0
            self._reference_marker.color.g = 1.0  # Green when active
            self._reference_marker.color.b = 0.0
            self._local_target_marker.color.a = 1.0
            self._local_target_marker.color.r = 0.0
            self._local_target_marker.color.g = 1.0  # Green when active
            self._local_target_marker.color.b = 0.0
            self._marker_pub.publish(self._marker_array)

    def set_succeeded(self, goal_handle):  # noqa: W613 # pylint: disable=W0613
        """
        Set a goal as succeeded and cleans everything up.

        @param  goal_handle The goal to process
        @return result (ActionResult): The result to be published
        """
        if self._halt_on_completion.value:
            if self._cmd_pub:
                self._cmd_pub.publish(Twist())

            if self._cmd_pub_stamped:
                twst = TwistStamped()  # Zero twist to stop if blocked
                twst.header.stamp = self.get_clock().now().to_msg()  # update the time stamp
                self._cmd_pub_stamped.publish(twst)

        # self._server_goal.set_succeeded(result)
        if self._marker_pub:
            self._reference_marker.color.a = 0.0  # Hide on success
            self._reference_marker.color.r = 0.0
            self._reference_marker.color.g = 0.0
            self._reference_marker.color.b = 1.0  # Blue on success
            self._local_target_marker.color.a = 0.0  # Hide on success
            self._local_target_marker.color.r = 0.0
            self._local_target_marker.color.g = 0.0
            self._local_target_marker.color.b = 1.0  # Blue on success
            self._marker_pub.publish(self._marker_array)

        result = self._result_type()
        result.code = self._result_type.SUCCESS
        if self._target is not None:
            result.pose.header = self._target.header
            result.pose.pose.position = self._target.point  # Using final target pose NOT robot pose!

        self._target = None
        self._prior = None
        goal_handle.succeed()
        self.get_logger().info("set_success!")
        return result

    def set_failed(self, goal_handle):  # noqa: W613 # pylint: disable=W0613
        """
        Set a goal as aborted and cleans everything up.

        @param  goal_handle The goal to process
        @return result (ActionResult): The result to be published
        """
        if self._halt_on_failure.value:
            if self._cmd_pub:
                self._cmd_pub.publish(Twist())

            if self._cmd_pub_stamped:
                twst = TwistStamped()  # Zero twist to stop if blocked
                twst.header.stamp = self.get_clock().now().to_msg()  # update the time stamp
                self._cmd_pub_stamped.publish(twst)

        # self._server_goal.set_failed(result)
        if self._marker_pub:
            self._reference_marker.color.a = 1.0
            self._reference_marker.color.r = 1.0  # red on failure
            self._reference_marker.color.g = 0.0
            self._reference_marker.color.b = 0.0  # Blue on success
            self._local_target_marker.color.a = 1.0
            self._local_target_marker.color.r = 1.0  # red on failure
            self._local_target_marker.color.g = 0.0
            self._local_target_marker.color.b = 0.0  # Blue on success
            self._marker_pub.publish(self._marker_array)

        result = self._result_type()
        result.code = self._result_type.FAILURE
        if self._target is not None:
            result.pose.header = self._target.header
            result.pose.pose.position = self._target.point  # Using final target pose NOT robot pose!
        goal_handle.abort()
        self.get_logger().info("set_failed!")
        self._target = None
        self._prior = None
        return result

    def set_canceled(self, goal_handle):  # noqa: W613 # pylint: disable=W0613
        """
        Set a goal as canceled and cleans everything up.

        @param  goal_handle The goal to process
        @return result (ActionResult): The result to be published
        """
        if self._halt_on_cancel.value:
            if self._cmd_pub:
                self._cmd_pub.publish(Twist())

            if self._cmd_pub_stamped:
                twst = TwistStamped()  # Zero twist to stop if blocked
                twst.header.stamp = self.get_clock().now().to_msg()  # update the time stamp
                self._cmd_pub_stamped.publish(twst)

        # self._server_goal.set_canceled(result)
        if self._marker_pub:
            self._reference_marker.color.a = 1.0
            self._reference_marker.color.r = 1.0  # red on failure
            self._reference_marker.color.g = 0.0
            self._reference_marker.color.b = 0.0  # Blue on success
            self._local_target_marker.color.a = 1.0
            self._local_target_marker.color.r = 1.0  # red on failure
            self._local_target_marker.color.g = 0.0
            self._local_target_marker.color.b = 0.0  # Blue on success
            self._marker_pub.publish(self._marker_array)

        result = self._result_type()
        result.code = self._result_type.CANCEL
        if self._target is not None:
            result.pose.header = self._target.header
            result.pose.pose.position = self._target.point  # Using final target pose NOT robot pose!

        goal_handle.canceled()
        self.get_logger().info("set_canceled!")
        self._target = None
        self._prior = None
        return result

    def pure_pursuit_control(self, goal_handle, desired_path):
        """
        Given starting index and desired path, process segments sequentially.

        @param goal_handle for action
        @param desired_path as nav_msgs.msg Path instance
        @return  None to continue processing, anything else to quit
        """
        # self.get_logger().info("Run pure_pursuit control calcs ...")
        self._control = None
        if not self.get_current_target(desired_path):
            if self._done:
                # In range of final point as fall back for success
                self.get_logger().info("In range of the final point as a fallback - done!")
                return 'done'

        if isinstance(self._control, PointStamped):
            curvature = 2.0 * self._control.point.y / self._lookahead_distance_squared
            self._twist.linear.x = self._desired_velocity.value
            self._twist.angular.z  = curvature * self._desired_velocity.value
            # self.get_logger().info(f"calculate at index={self._indice} - \n"
            #                        f"   control=(x={self._control.point.x:6.3f}, y={self._control.point.y:6.3f})\n"
            #                        f"    twist =(v={self._twist.linear.x:6.3f}, wz={self._twist.angular.z:6.3f})")

            if math.fabs(self._twist.angular.z) > self._max_rotation_rate.value:
                self._twist.linear.x  = (self._desired_velocity * self._max_rotation_rate.value
                                         / math.fabs(self._twist.angular.z))  # decrease the speed
                self._twist.angular.z = math.copysign(self._max_rotation_rate.value, self._twist.angular.z)

            if self._cmd_pub:
                self._cmd_pub.publish(self._twist)

            if self._cmd_pub_stamped:
                self._twist_stamped.header.stamp = self._target.header.stamp  # update the time stamp based on when calc relevant
                self._twist_stamped.twist.linear.x = self._twist.linear.x
                self._twist_stamped.twist.angular.z = self._twist.angular.z
                self._cmd_pub_stamped.publish(self._twist_stamped)

            if self._marker_pub:
                # Update the markers
                self._reference_marker.header = deepcopy(self._control.header)
                self._reference_marker.pose.position = deepcopy(self._control.point)
                self._local_target_marker.header = deepcopy(self._target.header)
                self._local_target_marker.pose.position = deepcopy(self._target.point)
                self._marker_pub.publish(self._marker_array)

            goal_handle.publish_feedback(self._feedback_type(twist=TwistStamped(header=self._target.header,  # noqa: E1102
                                                                                twist=self._twist),
                                                             pose=PoseStamped(header=self._target.header,
                                                                              pose=Pose(position=self._target.point))))
            return None  # Normal operation - continue along this path

        # Did not get a lookahead point so we either failed, completed segment, or
        # are deliberately holding the prior velocity (to recover from minor perturbations)
        self.get_logger().info(f"No lookahead at index={self._indice} - "
                               f"control={self._control} - local_prior={self._prior} hard fail!")
        return 'finished'  # could have failed or be done

    def find_furtherest_target(self, path):
        """
        Locate the goal point furtherest along the path from beginnng.

        We start from first point and search for a valid segment, and choose the last value.
        Start from beginning to allow closed loops

        @param poses (Pose[]): An array of poses for path
        @return index of last valid target point, or negative for invalid
        """
        index = 1
        valid_segment = False
        included_segment = False
        while index < len(path.poses) and not valid_segment and not included_segment:
            valid_segment, included_segment, _, _, _ = self.check_valid_segment(index, path)
            # self.get_logger().info(f" find_furtherest_target: valid_segment={valid_segment} at {index}")
            index += 1

        if index == len(path.poses):
            # self.get_logger().info(f"find_furtherest_target: No valid starting point at {index}")
            return -1  # did not find a valid starting point

        while index < len(path.poses) and (valid_segment or included_segment):
            valid_segment, included_segment, _, _, _ = self.check_valid_segment(index, path)
            # self.get_logger().info(f"find_furtherest_target: Still valid={valid_segment}"
            #                        f" or included={included_segment} starting point at {index}")
            index += 1

        if index == len(path.poses) and (valid_segment or included_segment):
            # all the way at end showing valid
            index -= 1
        else:
            index -= 2  # back to last valid

        valid_segment, included_segment, local_target, local_prior, control = self.check_valid_segment(index, path)

        self.get_logger().info(f"find_furtherest_target: Select starting point at {index}\n"
                               f" valid={valid_segment} included={included_segment} \n"
                               f"   target =(x={local_target.point.x :6.3f}, y={local_target.point.y:6.3f})\n"
                               f"   control=(x={control.point.x      :6.3f}, y={control.point.y:6.3f})\n"
                               f"   prior  =(x={local_prior.point.x  :6.3f}, y={local_prior.point.y:6.3f})")
        assert valid_segment
        return index  # index of target of last valid segment

    def check_valid_segment(self, index, path):
        """
        Check if segment is valid.

        Assumes a valid index
        """
        local_target = PointStamped(header=path.header, point=deepcopy(path.poses[index].pose.position))
        target_transform = self.transform_frame(local_target, self._robot_frame.value)
        local_target = None
        local_prior = None
        intersection = None
        valid_segment = False
        included_segment = False
        if target_transform is not None:
            local_target, target_transform = target_transform

            dr2 = local_target.point.x**2 + local_target.point.y**2  # Distance from robot reference
            if dr2 >= self._lookahead_distance_squared:
                # Found target across lookahead boundary
                local_prior = deepcopy(local_target)  # header is same
                local_prior.point = deepcopy(path.poses[index - 1].pose.position)

                p_ts = np.dot(target_transform,
                              np.array([local_prior.point.x, local_prior.point.y, local_prior.point.z, 1.0]).T)
                local_prior.point.x = p_ts[0]
                local_prior.point.y = p_ts[1]
                local_prior.point.z = p_ts[2]

                # Check to see if we have a valid intersection
                t1, t2, intersection = self.get_intersection_points(local_prior, local_target)

                valid_segment = t2 is not None and 0.0 <= t2 <= 1.0
                included_segment = None not in (t1, t2) and t1 < 0.0 and t2 > 1.0

        return valid_segment, included_segment, local_target, local_prior, intersection

    def get_current_target(self, path):
        """
        Locates the goal point to navigate to from a list of poses.

        Starts search at current indice and goes forward until the first segment that
        crosses the lookahead circle

        Sets self._target and self._prior in robot frame if successful

        @param poses (Pose[]): An array of poses for path
        @return True if successfully found valid segment, False otherwise
        """
        dr2 = 0.0
        self._target = None
        self._prior = None
        old_index = self._indice
        while dr2 < self._lookahead_distance_squared and self._running and rclpy.ok():
            if self._indice < len(path.poses):
                # Get target given valid index

                valid_segment, included_segment, local_target, local_prior, control = self.check_valid_segment(self._indice, path)
                if valid_segment:
                    self._target = local_target
                    self._prior = local_prior
                    self._control = control
                    if old_index != self._indice:
                        self.get_logger().info(f"Accepting index = {self._indice} of {len(path.poses)}")
                        # f" valid={valid_segment} included={included_segment} \n"
                        # f"   target =(x={self._target.point.x :6.3f}, y={self._target.point.y:6.3f})\n"
                        # f"   control=(x={self._control.point.x:6.3f}, y={self._control.point.y:6.3f})\n"
                        # f"   prior  =(x={self._prior.point.x  :6.3f}, y={self._prior.point.y:6.3f})")

                    return True

                if local_target is None:
                    self.get_logger().error(f"Could not get transformed target position from {path.header.frame_id}")
                    self._failed = True
                    return False

                # continue to the next segment and check
                if included_segment:
                    self.get_logger().info(f"Segment at index = {self._indice} of {len(path.poses)} is contained!\n")
                    #    f" valid={valid_segment} included={included_segment} \n"
                    #    f"   target =(x={self._target.point.x :6.3f}, y={self._target.point.y:6.3f})\n"
                    #    f"   control=(x={self._control.point.x:6.3f}, y={self._control.point.y:6.3f})\n"
                    #    f"   prior  =(x={self._prior.point.x  :6.3f}, y={self._prior.point.y:6.3f})")

            else:
                # Final check for success at end of line as we don't have any more segments to process
                local_target = PointStamped(header=path.header,
                                            point=deepcopy(path.poses[-1].pose.position))

                target_transform = self.transform_frame(local_target, self._robot_frame.value)

                if target_transform is None:
                    self.get_logger().error("Could not get transformed final target position"
                                            f" from {local_target.header.frame_id}")
                    self._failed = True
                    return False

                local_target = target_transform[0]  # get the transformed point
                dr2 = local_target.point.x**2 + local_target.point.y**2  # Distance from robot reference
                if dr2 < self._lookahead_distance_squared:
                    # Final target is within lookahead distance
                    self.get_logger().info('%s final target index =%d = %d target=(%f,%f) dr2=%f'
                                           % (self.get_name(), self._indice, len(path.poses),
                                              local_target.point.x, local_target.point.y, dr2)
                                           )
                    self._target = local_target
                    self._done = True
                    return False  # still false as not valid segment for control

                # Failed to find valid index
                self.get_logger().info('%s Failed to find target index =%d >= %d (old=%d) target=(%f,%f) '
                                       % (self.get_name(), self._indice, len(path.poses), old_index,
                                          local_target.point.x, local_target.point.y)
                                       )
                if -1 < old_index < len(path.poses):
                    old_target = PointStamped(header=path.header,
                                              point=deepcopy(path.poses[old_index].pose.position))

                    old_target_transform = self.transform_frame(old_target, self._robot_frame.value)
                    if old_target_transform is None:
                        self.get_logger().info('%s Failed to find old target index =%d >= %d  target=(%f,%f) '
                                               % (self.get_name(), old_index, len(path.poses),
                                                  local_target.point.x, local_target.point.y))
                    else:
                        old_target = old_target_transform[0]
                        valid_segment, included_segment, local_target, local_prior, control = \
                            self.check_valid_segment(old_index, path)

                        self.get_logger().info('%s  old target index =%d >= %d  target=(%f,%f) '
                                               'check target=(%f,%f) valid=(%d, %d)'
                                               % (self.get_name(), old_index, len(path.poses),
                                                  old_target.point.x, old_target.point.y,
                                                  local_target.point.x, local_target.point.y,
                                                  valid_segment, included_segment))

                self._failed = True
                return False

            self._indice += 1  # Check next index

        self.get_logger().error(f'{self.get_name()} Invalid get_current_error - how did we get here?')
        self._failed = True
        return False

    def get_intersection_points(self, local_prior, local_target):
        """Calculate terms in quadratic equation to determine circle-line intersection."""
        # Define a line segment from prior to the target (assume 2D)
        pv = Vector3(x=local_target.point.x - local_prior.point.x,
                     y=local_target.point.y - local_prior.point.y, z=0.0)

        # Vector from robot body reference to first point
        qv = Vector3(x=local_prior.point.x, y=local_prior.point.y, z=0.0)

        # Find intersection of line segment with lookahead circle centered at the  robot
        a = pv.x * pv.x + pv.y * pv.y
        b = 2.0 * (qv.x * pv.x + qv.y * pv.y)
        c = qv.x * qv.x + qv.y * qv.y - self._lookahead_distance_squared
        discrim = b * b - 4 * a * c
        t1 = None
        t2 = None
        control = None

        if a > 0.0001 and discrim >= 0.0:
            # solve quadratic equation for intersection points
            sqd = math.sqrt(discrim)
            t1 = (-b - sqd) / (2 * a)  # min value
            t2 = (-b + sqd) / (2 * a)  # max value

            # Define the control lookahead point in the robot's frame
            control = deepcopy(local_prior)
            control.point.x = control.point.x + t2 * pv.x  # Control point in the robot frame
            control.point.y = control.point.y + t2 * pv.y  # Control point in the robot frame
            control.point.z = control.point.z + t2 * pv.z  # Control point in the robot frame

        return t1, t2, control

    # Transform point into map frame
    def transform_frame(self, point, frame_id):
        """
        Transform point from one frame to target frame_id.

        @param point - location with header defining current frame_id
        @param target_frame - which frame to represent
        @return timestamped point in new frame given latest available transform
        """
        # Largely duplicated from pure_pursuit_state, but keep independent
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

        except (tf2_ros.ExtrapolationException) as exc:
            self.get_logger().warn(f'Failed to get the transformation from {point.header.frame_id} to '
                                   f'target_frame {frame_id} at t={point.header.stamp} within '
                                   f'{self._timeout} s now={self.get_clock().now().to_msg()}\n{type(exc)} {str(exc)}')
            return None
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException) as exc:
            self.get_logger().warn(f'Failed to get the transformation from {point.header.frame_id} to '
                                   f'target_frame {frame_id} at t={point.header.stamp} within '
                                   f'{self._timeout} s\n{type(exc)} {str(exc)}')
            return None
        except (TypeError, tf2_ros.buffer_interface.TypeException) as exc:
            # https://github.com/ros2/geometry2/issues/110
            msg = (f'Failed to get the transformation from {point.header.frame_id} to {frame_id} frame due to '
                   f' type conversion error\n {type(exc)} {str(exc)}')
            msg = msg.replace('<', "[")  # Avoid string format issues with logger
            msg = msg.replace('>', "]")
            self.get_logger().warn(msg)
            self.get_logger().info("See note in CHANGELOG")
            return None
        except Exception as exc:  # pylint: disable=W0703
            msg = (f'Failed to get the transformation from {point.header.frame_id} to {frame_id} frame '
                   f'due to unknown error\n {type(exc)} {str(exc)}')
            msg = msg.replace('<', "[")  # Avoid string format issues with logger
            msg = msg.replace('>', "]")
            self.get_logger().warn(msg)
            trace = traceback.format_exc()
            self.get_logger().info(' --------------------- Trace ------------------------------')
            self.get_logger().info(f''' Trace: {trace.replace("%", "%%")}''')
            self.get_logger().info(' --------------------- Trace ------------------------------')
            return None

    # def odom_cb(self, data):
    #     """
    #     The call back for the odometry subscriber

    #     Args:
    #         data (Odometry): The odometry message to process
    #     """

    #     self._last_odom_msg = data

    @staticmethod
    def _make_transform(posn, quat):
        """
        Given translation and orientation in normalized quaternion, return a 4x4 homogeneous transform matrix.

        https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/index.htm

        Make use of Python duck typing to work with several message forms

        @return point in new frame, transformation matrix
        """
        trans = np.zeros((4, 4))

        # Rotation matrix (by column)
        trans[0][0] = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        trans[1][0] = 2 * (quat.x * quat.y + quat.z * quat.w)
        trans[2][0] = 2 * (quat.x * quat.z - quat.y * quat.w)

        trans[0][1] = 2 * (quat.x * quat.y - quat.z * quat.w)
        trans[1][1] = 1 - 2 * (quat.x * quat.x + quat.z * quat.z)
        trans[2][1] = 2 * (quat.y * quat.z + quat.x * quat.w)

        trans[0][2] = 2 * (quat.x * quat.z + quat.y * quat.w)
        trans[1][2] = 2 * (quat.y * quat.z - quat.x * quat.w)
        trans[2][2] = 1 - 2 * (quat.x * quat.x + quat.y * quat.y)

        # Translation
        trans[0][3] = posn.x
        trans[1][3] = posn.y
        trans[2][3] = posn.z
        trans[3][3] = 1.0

        return trans
