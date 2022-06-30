cmake_minimum_required(VERSION 3.0.2)
project(object_detection)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  cv_bridge
  message_generation
)

find_package(OpenCV REQUIRED)
find_package(Eigen3  REQUIRED)
find_package(objection_detection)
## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   Object.msg
   Objects.msg
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   sensor_msgs
   std_msgs
 )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES object_detection
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs cv_bridge
  DEPENDS system_lib message_runtime
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include
        ${EIGEN3_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)

#CUDA
option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
find_package(CUDA REQUIRED)
set(CUDA_NVCC_PLAGS ${CUDA_NVCC_PLAGS};-std=c++11;-g;-G;-gencode;arch=compute_30;code=sm_30)
# include and link dirs of cuda and tensorrt, you need adapt them if yours are different
# cuda
include_directories(/usr/local/cuda/include)
link_directories(/usr/local/cuda/lib64)
# tensorrt
include_directories(/usr/include/x86_64-linux-gnu/) #aarch64-linux-gnu #x86_64-linux-gnu/
link_directories(/usr/lib/x86_64-linux-gnu/)
cuda_add_library(myplugins SHARED include/object_detection/yololayer.cu)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -Ofast -Wfatal-errors -D_MWAITXINTRIN_H_INCLUDED")

add_executable(${PROJECT_NAME}_node src/detector.cpp  src/yolo_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES} ${OpenCV_LIBS}  nvinfer cudart myplugins
        )
