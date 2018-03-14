cmake_minimum_required(VERSION 2.8.3)
SET( CMAKE_CXX_FLAGS "-std=c++11 -O3")

project(force_tracking)

set(CMAKE_PREFIX_PATH  ${CMAKE_PREFIX_PATH} $ENV{QT5_INSTALL_PATH})

find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  roscpp
  sensor_msgs
  trac_ik_lib
  trajectory_msgs
  rosbag
)


catkin_package(
  INCLUDE_DIRS include
)


include_directories(include
  ${catkin_INCLUDE_DIRS}
)

#add_executable(ur_force_teaching_node node/ur_force_teaching_node.cpp src/ur_force_teaching.cpp simulink2C/end_position/end_position_filter.cpp simulink2C/force_loop/force_loop.cpp)
add_executable(ur_force_teaching_node node/ur_force_teaching_node.cpp src/ur_force_teaching.cpp simulink2C/tracking/force_loop.cpp)
target_link_libraries(ur_force_teaching_node ${catkin_LIBRARIES})

