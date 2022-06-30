cmake_minimum_required(VERSION 2.8.3)
project(cams_recorder)

find_package(catkin REQUIRED COMPONENTS  roscpp std_msgs  sensor_msgs cv_bridge image_transport)
find_package(OpenCV  REQUIRED)
set(CMAKE_CXX_FLAGS "-std=c++11") # -std=c++11 to -std=gnu++11

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose Release or Debug" FORCE)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fpermissive" )

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS roscpp sensor_msgs std_msgs cv_bridge image_transport
        DEPENDS system_lib
)
###########
## Build ##
###########

include_directories(include
        ${OpenCV_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(${PROJECT_NAME}_node nodes/usb_cam_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
         ${catkin_LIBRARIES} ${OpenCV_LIBS}
        )

