cmake_minimum_required(VERSION 2.8.3)
project(flex_nav_common)

find_package(catkin REQUIRED
	COMPONENTS
		actionlib
		actionlib_msgs
		genmsg
		geometry_msgs
		message_generation
		message_runtime
		nav_core
		nav_msgs
		roscpp
		std_msgs
)

add_action_files(
	DIRECTORY
		action
	FILES
		ClearCostmap.action
		FollowPath.action
		FollowTopic.action
		GetPath.action
)

generate_messages(
	DEPENDENCIES
		actionlib_msgs
		geometry_msgs
		nav_msgs
		std_msgs
)

catkin_package(
	CATKIN_DEPENDS
		actionlib_msgs
		geometry_msgs
		message_runtime
		nav_core
		nav_msgs
		roscpp
		std_msgs
)
