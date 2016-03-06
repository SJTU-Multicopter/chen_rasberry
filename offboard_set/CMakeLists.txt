cmake_minimum_required(VERSION 2.8.3)
project(offboard_set)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES offboard_set
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(receive_setpoints src/receive_setpoints.cpp)  
target_link_libraries(receive_setpoints ${catkin_LIBRARIES}) 


add_executable(process_setpoints src/process_setpoints.cpp)  
target_link_libraries(process_setpoints ${catkin_LIBRARIES})   

#add_executable(setpoints_publisher src/setpoints_publisher.cpp)  
#target_link_libraries(setpoints_publisher ${catkin_LIBRARIES})  
#add_dependencies(setpoints_publisher beginner_tutorials_generate_messages_cpp)


