cmake_minimum_required(VERSION 2.8.3)
project(robot_container)

add_definitions("-std=c++11")

find_package(catkin REQUIRED
  COMPONENTS
    ros_node_utils
    message_generation
    std_msgs
    ros_node_utils
    roscpp
    actionlib_msgs
    actionlib
    trajectory_msgs
)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)


add_action_files(
  DIRECTORY
    action
  FILES
    ControllerTrajectory.action
)

add_service_files(
  FILES
    SetState.srv
)
add_message_files(
  FILES
    State.msg
)
generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs   trajectory_msgs
)

catkin_package(
	INCLUDE_DIRS
	  include
	  ${EIGEN3_INCLUDE_DIR}
	  ${boost_INCLUDE_DIR}
	LIBRARIES
	CATKIN_DEPENDS
	  ros_node_utils
	  roscpp
	  message_generation
	  std_msgs
    trajectory_msgs
    actionlib_msgs
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
