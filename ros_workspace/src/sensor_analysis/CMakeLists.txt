cmake_minimum_required(VERSION 3.0.2)
project(sensor_analysis)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  #message_generation
)

#add_message_files(
#  FILES
#  LaserDistances.msg
#  LaserAngles.msg
#)

#generate_messages(
#  DEPENDENCIES
#  std_msgs
#)

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp sensor_msgs std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(laser_scan_processor src/laser_scan_processor.cpp)
target_link_libraries(laser_scan_processor ${catkin_LIBRARIES})
#add_dependencies(laser_scan_processor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
