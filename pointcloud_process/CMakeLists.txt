cmake_minimum_required(VERSION 3.0.2)
project(pointcloud_process)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
#set(PCL_DIR "/usr/local/share/pcl-1.9") ##for ros-noetic in ubuntu 20.04 ,default pcl version is pcl 1.10 ,must be set to 1.9

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  pcl_ros
  tf
)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS include
  LIBRARIES pointcloud_process
  CATKIN_DEPENDS ros_pcl roscpp rospy
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
	${PCL_INCLUDE_DIRS}
)

add_executable(pointcloud_node  src/pointsCorrect.cpp src/map.cpp src/matrix.cpp  src/pointcloud_node.cpp )
add_executable(costmap_node  src/pointsCorrect.cpp src/map.cpp src/matrix.cpp   src/costmap_node.cpp)
target_link_libraries(pointcloud_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(costmap_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

