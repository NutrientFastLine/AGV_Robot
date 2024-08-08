cmake_minimum_required(VERSION 3.0.2)
project(base_control)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  serial
  std_msgs
  tf
)

#
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES base_control
  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS system_lib
)
include_directories(
  /usr/local/include/modbus
  ${catkin_INCLUDE_DIRS}
)
add_executable(base_control src/base_control.cpp) 
add_dependencies(base_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}) 
target_link_libraries(base_control ${catkin_LIBRARIES})
add_executable(DS2024_motor_driver_node src/DS2024_motor_driver_node.cpp) 
target_link_libraries(DS2024_motor_driver_node ${catkin_LIBRARIES})
target_link_libraries(DS2024_motor_driver_node ${catkin_LIBRARIES} modbus)
