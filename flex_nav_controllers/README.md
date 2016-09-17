Flexible Navigation Controller Nodes
====================================

The controller nodes are all implementations of an [ActionServer] and will
control your robot to traverse a path.

Overview:
---------

The `follow_path` and `follow_topic` nodes are both implementations of
[base_local_planner]. They can be configured to use any local planner and will
publish [geometry_msgs]/[Twist] messages.

The `pure_pursuit_path` and `pure_pursuit_topic` nodes do not contain a planner
and only follow a [nav_msgs]/[Path].

[base_local_planner]: http://wiki.ros.org/base_local_planner
[geometry_msgs]: http://docs.ros.org/api/geometry_msgs/html/index-msg.html
[Twist]: http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
[nav_msgs]: http://docs.ros.org/api/nav_msgs/html/index-msg.html
[Path]: http://docs.ros.org/api/nav_msgs/html/msg/Path.html
[ActionServer]: http://wiki.ros.org/actionlib
