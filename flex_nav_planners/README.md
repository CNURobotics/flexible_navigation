Flexible Navigation Planner Nodes
=================================

The planner nodes are all implementations of an [ActionServer] and will generate
a [nav_msgs]/[Path].

Overview:
---------

The highest level planner is the `get_path` planner and it will return a [Path]
to any given [geometry_msgs]/[Pose]. The easiest way to do this is through
[rviz].

The lowest level planner is provided via a controller. See the `controllers`
package for more information.

In addition to the standard global and local planners, you may also deploy any
number of "mid-level" planners. The `follow_path` and `follow_topic` nodes have
been designed to accept a [Path] and generate another [Path] from a different
costmap with potentially higher accuracy. You may chain several planners
together by using the `follow_topic` node and instructing them to listen to one
another.

[ActionServer]: http://wiki.ros.org/actionlib
[nav_msgs]: http://docs.ros.org/api/nav_msgs/html/index-msg.html
[Path]: http://docs.ros.org/api/nav_msgs/html/msg/Path.html
[geometry_msgs]: http://docs.ros.org/api/geometry_msgs/html/index-msg.html
[Pose]: http://docs.ros.org/api/geometry_msgs/html/msg/Pose.html
[rviz]: http://wiki.ros.org/rviz
