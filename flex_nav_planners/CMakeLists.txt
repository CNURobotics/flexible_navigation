cmake_minimum_required(VERSION 3.5)
project(flex_nav_planners)

find_package(ament_cmake REQUIRED)
find_package(flex_nav_common REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_navfn_planner REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  flex_nav_common
  std_msgs
  nav2_core
	nav2_msgs
  nav2_util
  geometry_msgs
)

include_directories(
	include
)

# flex_planner_follow_path node
add_executable(${PROJECT_NAME}_follow_path_node
	src/follow_path_node.cpp
	src/follow_path.cpp
	src/follow_common.cpp
)
ament_target_dependencies(${PROJECT_NAME}_follow_path_node
	${dependencies})

# flex_planner_follow_topic node
add_executable(${PROJECT_NAME}_follow_topic_node
	src/follow_topic_node.cpp
	src/follow_topic.cpp
	src/follow_common.cpp
)
ament_target_dependencies(${PROJECT_NAME}_follow_topic_node
	${dependencies})

# flex_planner_get_path node
add_executable(${PROJECT_NAME}_get_path_node
	src/get_path_node.cpp
	src/get_path.cpp
)

ament_target_dependencies(${PROJECT_NAME}_get_path_node
	${dependencies})

install(
	TARGETS
		${PROJECT_NAME}_follow_path_node
		${PROJECT_NAME}_follow_topic_node
		${PROJECT_NAME}_get_path_node
		RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION include//${PROJECT_NAME}
	FILES_MATCHING PATTERN "*.h"
)

ament_package()
