cmake_minimum_required(VERSION 3.0.2)
project(fusion_perception)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

#set(PCL_DIR "/usr/local/share/pcl-1.9") ##for ros-noetic in ubuntu 20.04 ,default pcl version is pcl 1.10 ,must be set to 1.9

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  message_generation
)

find_package(OpenCV REQUIRED)
find_package(Eigen3  REQUIRED)
find_package(PCL REQUIRED)

add_message_files(
    FILES
    Image.msg
    Frame.msg
    Track.msg
)


## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   geometry_msgs  sensor_msgs  std_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES fusion_perception
  CATKIN_DEPENDS cv_bridge geometry_msgs roscpp sensor_msgs std_msgs
  DEPENDS system_lib message_runtime
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_executable(fusion_node  src/fusion_node.cpp include/tracker/tracker2d.hpp)
target_link_libraries(fusion_node ${catkin_LIBRARIES}  ${PCL_LIBRARIES} ${OpenCV_LIBS} )

