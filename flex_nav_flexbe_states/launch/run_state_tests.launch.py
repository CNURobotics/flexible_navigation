from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


flexbe_testing_dir = get_package_share_directory('flexbe_testing')
flex_nav_states_test_dir = get_package_share_directory('flex_nav_flexbe_states')

path = flex_nav_states_test_dir + "/test"

testcases  = path + "/get_path_state.test \n"
testcases += path + "/log_path_state.test \n"
testcases += path + "/pure_pursuit_state.test \n"
testcases += path + "/timed_twist_state.test \n"
testcases += path + "/timed_stop_state.test \n"
testcases += path + "/clear_costmaps_state.test \n"
testcases += path + "/follow_path_state.test \n"
testcases += path + "/follow_topic_state.test \n"
testcases += path + "/follow_planner_state.test \n"
testcases += path + "/move_distance_state.test \n"
testcases += path + "/rotate_angle_state.test \n"
testcases += path + "/recovery_state.test"


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("compact_format", default_value="True"),
        DeclareLaunchArgument("testcases", default_value=testcases),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(flexbe_testing_dir + "/launch/flexbe_testing.launch.py"),
            launch_arguments={
                'compact_format': LaunchConfiguration("compact_format"),
                'testcases': LaunchConfiguration("testcases"),
            }.items()
        )
    ])
