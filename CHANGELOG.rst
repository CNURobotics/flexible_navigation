^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package flexible_navigation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
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
However, there are issues with Pure Pursuit due to TF2 issue #110. This issue prevents the
transforming of Point and PointStamped messages to different frames such as
the robot pose obtained from the odom frame not being transformed to the map frame.
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
