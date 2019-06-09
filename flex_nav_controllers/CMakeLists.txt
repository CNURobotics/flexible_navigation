cmake_minimum_required(VERSION 2.8.3)
project(flex_nav_controllers)

find_package(catkin REQUIRED
	COMPONENTS
		actionlib
		actionlib_msgs
		flex_nav_common
		geometry_msgs
		message_generation
		message_runtime
		nav_core
		nav_msgs
		roscpp
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

catkin_python_setup()

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

# flex_controller_follow_path node
add_executable(${PROJECT_NAME}_follow_path_node
	src/follow_path_node.cpp
	src/follow_path.cpp
)
add_dependencies(${PROJECT_NAME}_follow_path_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_follow_path_node
	${Boost_LIBRARIES}
	${catkin_LIBRARIES}
)

# flex_controller_follow_topic node
add_executable(${PROJECT_NAME}_follow_topic_node
	src/follow_topic_node.cpp
	src/follow_topic.cpp
)
add_dependencies(${PROJECT_NAME}_follow_topic_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_follow_topic_node
	${Boost_LIBRARIES}
	${catkin_LIBRARIES}
)

install(
	TARGETS
		${PROJECT_NAME}_follow_path_node
		${PROJECT_NAME}_follow_topic_node
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
)

# Install Python executables
install(PROGRAMS bin/${PROJECT_NAME}_pure_pursuit_path
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS bin/${PROJECT_NAME}_pure_pursuit_topic
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
