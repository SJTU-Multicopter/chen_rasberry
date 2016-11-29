cmake_minimum_required(VERSION 2.8.3)
project(lidar_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Lidar.msg
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES kalman_filter
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(tf20_lib
  src/tf20/tf20.cpp
)


## Declare a C++ executable
add_executable(tf20_node src/tf20/tf20_node.cpp)
target_link_libraries(tf20_node
  ${catkin_LIBRARIES}
  tf20_lib
)

SET(CMAKE_C_FLAGS  "-DLT_LINUX -std=c99 -fmax-errors=1")
add_library(leddar_lib src/leddar_one/LeddarOne.c  src/leddar_one/Modbus.c  src/leddar_one/OS.c)

add_executable(leddar_node src/leddar_one/leddar_node.cpp)
target_link_libraries(leddar_node 
  ${catkin_LIBRARIES}
  leddar_lib
)
