cmake_minimum_required(VERSION 2.8.3)
project(ros_controller_base)

add_definitions("-std=c++11")

find_package(catkin REQUIRED
  COMPONENTS
    message_generation
    std_msgs
    ros_node_utils
    roscpp
    ooqp_eigen_interface
)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

add_message_files(
  FILES
    State.msg
    Input.msg
)
generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package(
	INCLUDE_DIRS
	  include
	  ${EIGEN3_INCLUDE_DIR}
	    ${boost_INCLUDE_DIR}
	LIBRARIES
	CATKIN_DEPENDS
	  roscpp
	  ros_node_utils
	  message_generation
	  std_msgs
	  ooqp_eigen_interface
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)


add_executable(controller_test
  test/test.cpp
)
add_dependencies(controller_test
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(controller_test
  ${catkin_LIBRARIES}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
