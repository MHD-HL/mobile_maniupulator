cmake_minimum_required(VERSION 2.8.3)
project(mybot_hardware_interface)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED
  hardware_interface
  controller_manager
  sensor_msgs
  serial
  roscpp
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS hardware_interface controller_manager sensor_msgs
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/mybot_hardware_interface.cpp src/mybot_hardware_interface_node.cpp)
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

