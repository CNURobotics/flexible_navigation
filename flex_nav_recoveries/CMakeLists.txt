cmake_minimum_required(VERSION 3.5)
project(flex_nav_recoveries)

find_package(ament_cmake REQUIRED)
find_package(flex_nav_common REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(navigation2 REQUIRED)
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
add_executable(${PROJECT_NAME}_recovery_server_node
	src/recovery_server_node.cpp
	src/recovery_server.cpp
)
ament_target_dependencies(${PROJECT_NAME}_recovery_server_node
	${dependencies})

install(
	TARGETS
		${PROJECT_NAME}_recovery_server_node
		RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION include//${PROJECT_NAME}
	FILES_MATCHING PATTERN "*.h"
)

ament_package()
