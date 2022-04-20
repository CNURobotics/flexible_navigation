#!/usr/bin/env python3

###############################################################################
#  Copyright (c) 2022
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
import yaml
from copy import deepcopy

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from flex_nav_common.action import GetPathByName
from nav_msgs.msg import Path
from geometry_msgs.msg import Point, PoseStamped, Quaternion

class GetPathByNameActionServer(Node):

    def __init__(self):
        super().__init__('get_path_by_name')
        self._action_server = ActionServer(
            self,
            GetPathByName,
            'get_path_by_name',
            self.execute_callback)
        self.declare_parameter('yaml_paths_file', "")
        self.declare_parameter('plan_topic', "plan")
        self.declare_parameter('max_distance', 0.25)

        plan_topic = self.get_parameter('plan_topic').get_parameter_value().string_value
        self._plan_publisher = self.create_publisher(Path, plan_topic, 1)

        self._paths_by_name = {"empty": Path()}

        self._max_distance  = self.get_parameter('max_distance').get_parameter_value().double_value
        self._yaml_file = self.get_parameter('yaml_paths_file').get_parameter_value().string_value
        if self._yaml_file != "":
            paths = self.load_paths_from_yaml(self._yaml_file)
            if paths:
                self._paths_by_name.update(paths)


    def get_pose_data(self, ndx, poses, current_frame_id):
        pose = poses[ndx]
        frame_id = current_frame_id
        if 'frame_id' in pose:
            frame_id = pose['frame_id']
            if frame_id != '' and current_frame_id != '':
                if current_frame_id != frame_id:
                    self.get_logger().error(f'Mismatched frame_id = {frame_id} {current_frame_id}!')
                    self.get_logger().error('   get_path_by_name does not handle transforms between frames!')

        position = np.array([pose['position']['x'], pose['position']['y'], 0.0])
        try:
            position[2] = pose['position']['z']
        except:
            pass # z-value is optional, stick with default 0.

        try:
            orientation=Quaternion(x=pose['orientation']['x'],
                                   y=pose['orientation']['y'],
                                   z=pose['orientation']['z'],
                                   w=pose['orientation']['w'])
        except:
            # orientation is optional
            orientation = None

        return position, orientation, frame_id

    def get_vector(self, prior_position, next_position):

        vector = next_position - prior_position
        if math.fabs(vector[2]) > 0.0005:
            self.get_logger().error(f'This code assumes 2D path! {prior_position} to {next_position}')

        magnitude = np.linalg.norm(vector)
        angle = math.atan2(vector[1], vector[0])
        orientation=Quaternion(x=0.,
                               y=0.,
                               z=math.sin(angle/2), # Assumes 2D !
                               w=math.cos(angle/2))
        return vector, magnitude, angle, orientation

    def load_paths_from_yaml(self, yaml_file):
        path_data = None
        try:
            with open(yaml_file, "rt") as fin:
                path_data = yaml.safe_load(fin)
        except yaml.YAMLError as exc:
            self.get_logger().error(f'Failed to load yaml data for paths by name from {yaml_file}')
            self.get_logger().error(str(exc))
            return None
        except (IOError, OSError) as exc:
            self.get_logger().error(f'Failed to open paths by name from {yaml_file}')
            return None

        if path_data:
            try:
                self.get_logger().info(f'Process paths by name from {yaml_file} ... ')
                paths_by_name = {}
                for name, data in path_data.items():

                    path = Path()
                    try:
                        frame_id = data['frame_id']
                    except:
                        frame_id = 'map' # Default frame ID if unspecified
                        self.get_logger().info(f'Using default frame_id = {frame_id}!')

                    try:
                        # Set up the initial pose information
                        path.header.frame_id = frame_id
                        poses = data['poses']
                        if len(poses) == 0:
                            self.get_logger().info(f"  Empty path defined for path {name} ...")
                            paths_by_name[name] = path
                            continue

                        self.get_logger().info(f"  Adding {len(poses)} poses to path {name} ...")
                        pose_frame_id = frame_id
                        prior_wp_posn = None
                        prior_wp_quat = None

                        # Get the starting point
                        prior_wp_posn, prior_wp_quat, prior_frame_id = self.get_pose_data(0, poses, pose_frame_id)
                        if pose_frame_id == '':
                            pose_frame_id = prior_frame_id

                        if len(poses) > 1:
                            position, orientation, frame_id = self.get_pose_data(1, poses, pose_frame_id)
                            vector, magnitude, angle, orientation = self.get_vector(prior_wp_posn, position)

                            if prior_wp_quat is None:
                                prior_wp_quat = orientation

                        next_wp_quat = prior_wp_quat
                        ros_pose = PoseStamped()
                        ros_pose.header.frame_id = pose_frame_id
                        ros_pose.pose.position = Point(x=prior_wp_posn[0], y=prior_wp_posn[1], z=prior_wp_posn[2])
                        ros_pose.pose.orientation = prior_wp_quat
                        path.poses.append(ros_pose) # Starting point

                        last_position = prior_wp_posn
                        next_wp_posn = prior_wp_posn
                        next_wp_quat = prior_wp_quat
                    except Exception as exc:
                        self.get_logger().error(f'Failed during processing of {yaml_file} for {name} @ initial pose')
                        self.get_logger().error(str(exc))
                        continue

                    for i in range(1, len(poses)):
                      try:
                        prior_wp_posn = next_wp_posn
                        prior_wp_quat = next_wp_quat
                        prior_angle  = angle
                        prior_vector = vector
                        prior_mag = magnitude

                        pose = poses[i]
                        next_wp_posn, next_wp_quat, next_frame_id = self.get_pose_data(i, poses, pose_frame_id)
                        vector, magnitude, angle, vec_orientation = self.get_vector(prior_wp_posn, next_wp_posn)

                        if next_wp_quat is None:
                            # Look at the average angle with next point
                            # This will possibly be used as starting point for
                            # the next segment
                            next_wp_quat = vec_orientation
                            if i < len(poses)-1:
                                follow_position, follow_orientation, follow_frame_id = self.get_pose_data(i+1, poses, pose_frame_id)
                                _, follow_magnitude, follow_angle, _ = self.get_vector(prior_wp_posn, follow_position)
                                mag_sum = magnitude + follow_magnitude
                                if mag_sum > 0.0001:
                                    # Shorter segments have MORE weight
                                    # unless they are trivially short as to
                                    # be ignored (e.g. duplicate points with ill defined angle)
                                    if magnitude/mag_sum < 0.02:
                                        next_angle = follow_angle
                                    elif magnitude/mag_sum > 0.98:
                                        next_angle = angle
                                    else:
                                        next_angle = (follow_magnitude*angle  + magnitude*follow_angle)/mag_sum

                                    next_wp_quat=Quaternion(x=0.,
                                                            y=0.,
                                                            z=math.sin(next_angle/2),
                                                            w=math.cos(next_angle/2))

                        if magnitude > 0.5*self._max_distance:
                            # Interpolate, otherwise use terminal as the next starting point
                            increment = self._max_distance/magnitude

                            # We are starting inside the vector, and then
                            # checking to see if we want a transition point
                            spacing = np.arange(0.25*increment, 1.0-0.24*increment, increment)

                            # Consider first interpolated point
                            new_position = prior_wp_posn + spacing[0]*vector
                            distance = np.linalg.norm(new_position - last_position)
                            if distance > 0.5*self._max_distance:
                                # Add intermediate point, to avoid large gap at
                                # piecewise junction
                                mid_position = prior_wp_posn
                                mid_orientation = prior_wp_quat
                                ros_pose = PoseStamped()
                                ros_pose.header.frame_id = pose_frame_id
                                ros_pose.pose.position = Point(x=mid_position[0], y=mid_position[1], z=mid_position[2])
                                ros_pose.pose.orientation = mid_orientation
                                path.poses.append(ros_pose)

                            for fraction in spacing:
                                position = prior_wp_posn + fraction*vector
                                ros_pose = PoseStamped()
                                ros_pose.header.frame_id = pose_frame_id
                                ros_pose.pose.position = Point(x=position[0], y=position[1], z=position[2])
                                ros_pose.pose.orientation = vec_orientation
                                path.poses.append(ros_pose)
                            last_position = position

                        elif i != len(poses)-1:
                            # Closely spaced point, just add the terminal point
                            ros_pose = PoseStamped()
                            ros_pose.header.frame_id = pose_frame_id
                            ros_pose.pose.position = Point(x=next_wp_posn[0], y=next_wp_posn[1], z=next_wp_posn[2])
                            ros_pose.pose.orientation = next_wp_quat
                            path.poses.append(ros_pose) # Starting point
                            last_position = next_wp_posn
                      except Exception as exc:
                          self.get_logger().error(f'Failed during processing of {yaml_file} @pose={i}')
                          self.get_logger().error(str(exc))
                          import traceback
                          self.get_logger().error(traceback.format_exc())


                    if len(poses) > 1:
                        # Add the final point into the path
                        ros_pose = PoseStamped()
                        ros_pose.header.frame_id = pose_frame_id
                        ros_pose.pose.position = Point(x=next_wp_posn[0], y=next_wp_posn[1], z=next_wp_posn[2])
                        ros_pose.pose.orientation = next_wp_quat
                        path.poses.append(ros_pose) # Starting point
                    paths_by_name[name] = path

                return paths_by_name

            except Exception as exc:
                self.get_logger().error(f'Failed to load yaml data for paths by name from {yaml_file}')
                self.get_logger().error(str(exc))
        else:
            return None

    def execute_callback(self, goal_handle):
        path_name = goal_handle._goal_request.path_name
        self.get_logger().info(f'Request path for {path_name}')
        result = GetPathByName.Result()
        result.code = GetPathByName.Result.FAILURE  # Assume the worst
        try:
            yaml_file = self.get_parameter('yaml_paths_file').get_parameter_value().string_value

            if yaml_file != self._yaml_file:
                self.get_logger().info(f'  Updating the paths by name data from {yaml_file}')
                paths = self.load_paths(self._yaml_file)
                if paths:
                    self._paths_by_name.update(paths)
                    self._yaml_file = yaml_file
                else:
                    self.get_logger().error(f'  No paths to update from {yaml_file}')

            self.get_logger().info(f'  Fetch path for {path_name}')
            path = deepcopy(self._paths_by_name[path_name])

            # Update the timing information
            self.get_logger().info(f'  type {type(path)} poses for  {path_name}')
            self.get_logger().info(f'  Got path with {len(path.poses)} poses for  {path_name}')
            time = self.get_clock().now().to_msg()
            path.header.stamp = time
            for pose in path.poses:
                pose.header.stamp = time

            self.get_logger().info(f'  Return resulting path for {path_name}')
            self._plan_publisher.publish(path)

            result.plan = path
            if len(path.poses) == 0:
                result.code = GetPathByName.Result.EMPTY
            else:
                result.code = GetPathByName.Result.SUCCESS
            goal_handle.succeed()

        except KeyError as exc:
            self.get_logger().error(f'Failed to retrieve path by name {path_name} - KeyError')
            self.get_logger().error(exc)
            available_paths = [key for key in self._paths_by_name]
            self.get_logger().error(f'   Available paths: {available_paths} ')
            goal_handle.abort()

        except Exception as exc:
            self.get_logger().error(f'Unknown Exception - Failed to retrieve path by name {path_name}')
            self.get_logger().error(f'{exc}')
            available_paths = [key for key in self._paths_by_name]
            self.get_logger().error(f'   Available paths: {available_paths} ')
            goal_handle.abort()

        return result


def main(args=None):
    rclpy.init(args=args)

    action_server = GetPathByNameActionServer()

    rclpy.spin(action_server)


if __name__ == '__main__':
    main()
