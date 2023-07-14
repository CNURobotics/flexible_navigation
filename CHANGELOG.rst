^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package flexible_navigation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
0.3.2 (2022-08-02)
Humble release
  * renamed flex_nav_recoveries to more general flex_nav_behaviors to allow standalone BT use
  * identify proxy subscribers by process instance id
  * handle created/destroy Bond and nodeOptions
  * PurePursuit not working due to https://github.com/ros2/geometry2/issues/110
    * Modified to use common transform handling with custom exception messages
  * Tested under Ubuntu 22.04 and ROS 2 Humble
---------------------
0.3.1
Fix state tests
Add get_path_by_name state and server
Add set_pose_state
Modify states and nodes to publish cmd_vel as Twist and cmd_vel_stamped as TwistStamped
  * May choose either or both, but defaults to Twist
Tested under ROS 2 Foxy
---------------------
0.3.0 Update for conversion to ROS2 Foxy
Currently, able to execute all flex_nav_controllers and flex_nav_planners nodes.
However, there are issues with Pure Pursuit due to TF2 issue #110.
https://github.com/ros2/geometry2/issues/110
This issue prevents the transforming of Point and PointStamped messages to different frames
such as the robot pose obtained from the odom frame not being transformed to the map frame.
Once, the issue is resolved and tf2_geometry_msgs Python version is released
then the Pure Pursuit nodes and states will be fully converted and executable.
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
0.2.0 Update for the latest Melodic updates
---------------------
0.1.1 Update for the latest Kinetic updates
---------------------
0.1.0 Modify for FlexBE App setup
---------------------
0.0.1 Initial release
