Flexible Navigation Controller Nodes
====================================

The controller nodes are all implementations of an [ActionServer] and will
control your robot to traverse a path. These controller nodes are based on the
ros-planning Navigation2 controller nodes.

Overview:
---------
The `follow_path` and `follow_topic` nodes are both implementations of
[base_local_planner] in ROS2. They can be configured to use any local planner and will
publish [geometry_msgs]/[Twist] messages.

The `pure_pursuit_path` and `pure_pursuit_topic` nodes do not contain a planner
and only follow a [nav_msgs]/[Path].

[Navigation2]: https://github.com/ros-planning/navigation2
[base_local_planner]: http://wiki.ros.org/base_local_planner
[geometry_msgs]: https://docs.ros2.org/foxy/api/geometry_msgs/index-msg.html
[Twist]: https://docs.ros2.org/foxy/api/geometry_msgs/msg/Twist.html
[nav_msgs]: https://docs.ros2.org/foxy/api/nav_msgs/index-msg.html
[Path]: https://docs.ros2.org/foxy/api/nav_msgs/msg/Path.html
[ActionServer]: http://design.ros2.org/articles/actions.html
