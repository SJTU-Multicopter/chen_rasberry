cmake_minimum_required(VERSION 2.8.3)
project(flowmeter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
#  geometry_msgs
  message_generation
  genmsg
)

add_message_files(
  FILES
  flowrate.msg
)

generate_messages(
   DEPENDENCIES
  std_msgs

)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

set(wiringPi_include "/usr/local/include")

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${wiringPi_include}
)
add_executable(flowmeter src/flowmeter.cpp)
target_link_libraries(flowmeter
  ${catkin_LIBRARIES} 
  wiringPi)
add_dependencies(flowmeter flowmeter_cpp)
