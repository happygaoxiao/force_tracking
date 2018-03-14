# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(fake_etherdaq)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs )


catkin_package(

)
## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(fake_force src/fake_force.cpp)
target_link_libraries(fake_force ${catkin_LIBRARIES})


# %EndTag(FULLTEXT)%