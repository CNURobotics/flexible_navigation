Flexible Navigation Planner Nodes
=================================

The planner nodes are all implementations of an [ActionServer] and will generate
a [nav_msgs]/[Path] based on the ros-planning Navigation2 planner nodes.

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

[Navigation2]: https://github.com/ros-planning/navigation2
[ActionServer]: http://design.ros2.org/articles/actions.html
[nav_msgs]: https://docs.ros2.org/foxy/api/nav_msgs/index-msg.html
[Path]: https://docs.ros2.org/foxy/api/nav_msgs/msg/Path.html
[geometry_msgs]: https://docs.ros2.org/foxy/api/geometry_msgs/index-msg.html
[Pose]: https://docs.ros2.org/foxy/api/geometry_msgs/msg/Pose.html
[rviz]: https://github.com/ros2/rviz
