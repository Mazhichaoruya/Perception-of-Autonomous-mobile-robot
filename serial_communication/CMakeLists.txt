cmake_minimum_required(VERSION 3.0.2)
project(serial_communication)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  geometry_msgs
  nav_msgs
  tf
  serial
)

## Generate messages in the 'msg' folder
add_message_files(FILES
  sensor_data.msg
  control_command.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES serial_communication
 CATKIN_DEPENDS roscpp std_msgs message_generation message_runtime geometry_msgs nav_msgs tf serial 
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(serial_communication_node 
  src/serial_node.cpp)
target_link_libraries(serial_communication_node
        ${catkin_LIBRARIES}
        )
add_executable(telop_key_node src/telop_key.cpp)
target_link_libraries(telop_key_node
        ${catkin_LIBRARIES}
        )