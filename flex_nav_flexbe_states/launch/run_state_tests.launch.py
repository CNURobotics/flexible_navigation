from os.path import join

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
def generate_launch_description():

    flexbe_testing_dir = get_package_share_directory('flexbe_testing')
    flex_nav_states_test_dir = get_package_share_directory('flex_nav_flexbe_states')

    path = join(flex_nav_states_test_dir, "test")

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
        DeclareLaunchArgument("pkg", default_value="flex_nav_flexbe_states"), #flexbe_testing"),
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
