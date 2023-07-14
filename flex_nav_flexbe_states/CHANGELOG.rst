^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package flex_nav_flexbe_states
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2023-07-14)
------------------
* flake8, pylint, cpplint, copyright cleanup
* Humble release
* do an emergency stop if behavior pauses/stops while action is navigating
* correct missed pub_stamp
* single ProxyPublisher initialized
* new get_path_by_name state and server; set_pose_state; cmd_vel output type change and options; state test fix
* ROS 2 Foxy Cleanup
* ROS 2 Foxy release
* conversion to ROS Melodic
* update package depends; latest kinetic setup
* set timestamp properly; handle blocked execution by storing return values; add timed twist, stop, and pure pure pursuit states
* add operator hint style and information
* modify for the new FlexBE app setup
* remove unused CreateSensorState.msg
* Un-hardcode pose topic
  See Task 74.
* make version consistent with other packages in flexible_navigation
* Initial commit
  Flexible Navigation v1.0.0
