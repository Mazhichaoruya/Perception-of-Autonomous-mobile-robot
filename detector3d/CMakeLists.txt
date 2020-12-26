cmake_minimum_required(VERSION 3.0.2)
project(detector3d)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
)

find_package(OpenCV 3 REQUIRED )
#find_package(OpenCV REQUIRED
#PATHS /usr/local/share # look here, mat /usr/local
#NO_DEFAULT_PATH	)
include_directories( ${OpenCV_INCLUDE_DIRS} )
#[[boost]]
#find_package(Boost REQUIRED COMPONENTS system thread)
#include_directories(${Boost_INCLUDE_DIRS})
# eigen
include_directories( "/usr/include/eigen3/" )
# Find librealsense2 installed package
#find_package(realsense2 REQUIRED)
#add_subdirectory(./dlib dlib_build)
find_package(PCL REQUIRED)
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
#CUDA
option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
#set(CMAKE_BUILD_TYPE Debug)
find_package(CUDA REQUIRED)
set(CUDA_NVCC_PLAGS ${CUDA_NVCC_PLAGS};-std=c++11;-g;-G;-gencode;arch=compute_30;code=sm_30)
# include and link dirs of cuda and tensorrt, you need adapt them if yours are different
# cuda
include_directories(/usr/local/cuda/include)
link_directories(/usr/local/cuda/lib64)
# tensorrt
include_directories(/usr/include/x86_64-linux-gnu/)
link_directories(/usr/lib/x86_64-linux-gnu/)
cuda_add_library(myplugins SHARED ${PROJECT_SOURCE_DIR}/include/yololayer.cu)
target_link_libraries(myplugins nvinfer cudart)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -Ofast -Wfatal-errors -D_MWAITXINTRIN_H_INCLUDED")

## Generate messages in the 'msg' folder
add_message_files(
        FILES
        object.msg
        objectsofonemat.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
        std_msgs
        geometry_msgs
        sensor_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES detector3d
  CATKIN_DEPENDS cv_bridge geometry_msgs image_transport roscpp rospy sensor_msgs std_msgs message_runtime
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


## Add cmake target dependencies of the executable
## same as for the library above
add_executable(decpub src/detect.cpp  src/lidar_pointcloud.cpp src/kmeans.cpp src/depth_pointcloud.cpp) ##Yolov3_tiny.cpp  set_D435.cpp rs-D435.cpp rs-pointcloud.cpp src/Realsense_config.cpp src/GetCameraReferance.cpp src/Test.cpp
target_link_libraries(decpub ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} nvinfer cudart myplugins )#dlib::dlib   ${realsense2_LIBRARY} ${Boost_LIBRARIES}
#add_dependencies(realsense_d435_pub ${PROJECT_NAME}_generate_message_cpp)
add_definitions(-O2 -pthread)
add_executable(decdeal src/dealback.cpp)
target_link_libraries(decdeal ${catkin_LIBRARIES})
## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )