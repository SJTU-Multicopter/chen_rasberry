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

add_executable(offboard_setpoints src/offboard_setpoints.cpp)  
target_link_libraries(offboard_setpoints ${catkin_LIBRARIES})  
add_dependencies(offboard_setpoints beginner_tutorials_generate_messages_cpp)

add_executable(Height src/CalcHeight.cpp)
target_link_libraries(Height ${catkin_LIBRARIES})

#add_executable(offboard_velocity src/offboard_velocity.cpp)  
#target_link_libraries(offboard_velocity ${catkin_LIBRARIES})  
#add_dependencies(offboard_velocity beginner_tutorials_generate_messages_cpp)

add_executable(setpoints_publisher src/setpoints_publisher.cpp)  
target_link_libraries(setpoints_publisher ${catkin_LIBRARIES})  
add_dependencies(setpoints_publisher beginner_tutorials_generate_messages_cpp)

#add_executable(status_imitate src/status_imitate.cpp)  
#target_link_libraries(status_imitate ${catkin_LIBRARIES})  
#add_dependencies(status_imitate beginner_tutorials_generate_messages_cpp)

#add_executable(field_size_set src/field_size_set.cpp)  
#target_link_libraries(field_size_set ${catkin_LIBRARIES})  
#add_dependencies(field_size_set beginner_tutorials_generate_messages_cpp)
