#   Copyright (c) 2016-2023
#   Capable Humanitarian Robotics and Intelligent Systems Lab (CHRISLab)
#   Christopher Newport University
#
#   All rights reserved.
#
#   Redistribution and use in source and binary forms, with or without
#   modification, are permitted provided that the following conditions are met:
#
#     1. Redistributions of source code must retain the above copyright notice,
#        this list of conditions and the following disclaimer.
#
#     2. Redistributions in binary form must reproduce the above copyright
#        notice, this list of conditions and the following disclaimer in the
#        documentation and/or other materials provided with the distribution.
#
#     3. Neither the name of the copyright holder nor the names of its
#        contributors may be used to endorse or promote products derived from
#        this software without specific prior written permission.
#
#        THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#        "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#        LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#        FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#        COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#        INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#        BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#        LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#        CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#        LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
#        WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#        POSSIBILITY OF SUCH DAMAGE.


"""Run state tests via launch."""
from os.path import join

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch for state tests."""
    flexbe_testing_dir = get_package_share_directory('flexbe_testing')
    flex_nav_states_test_dir = get_package_share_directory('flex_nav_flexbe_states')

    path = join(flex_nav_states_test_dir, "tests")

    testcases = ""
    testcases += join(path, "clear_costmaps_state.test") + "\n"
    testcases += join(path, "follow_path_state.test") + "\n"
    testcases += join(path, "follow_planner_state.test") + "\n"
    testcases += join(path, "follow_topic_state.test") + "\n"
    testcases += join(path, "get_path_by_name_state.test") + "\n"
    testcases += join(path, "get_path_state.test") + "\n"
    testcases += join(path, "get_pose_state.test") + "\n"
    testcases += join(path, "log_path_state.test") + "\n"
    testcases += join(path, "move_distance_state.test") + "\n"
    testcases += join(path, "pure_pursuit_state.test") + "\n"
    testcases += join(path, "recovery_state.test") + "\n"
    testcases += join(path, "rotate_angle_state.test") + "\n"
    testcases += join(path, "set_indice_state.test") + "\n"
    testcases += join(path, "set_pose_state.test") + "\n"
    testcases += join(path, "timed_stop_state.test") + "\n"
    testcases += join(path, "timed_twist_state.test") + "\n"

    return LaunchDescription([
        DeclareLaunchArgument("pkg", default_value="flex_nav_flexbe_states"),
        DeclareLaunchArgument("testcases", default_value=testcases),
        DeclareLaunchArgument("compact_format", default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(join(flexbe_testing_dir, "launch", "flexbe_testing.launch.py")),
            launch_arguments={
                'package': LaunchConfiguration("pkg"),
                'compact_format': LaunchConfiguration("compact_format"),
                'testcases': LaunchConfiguration("testcases"),
            }.items()
        )
    ])
