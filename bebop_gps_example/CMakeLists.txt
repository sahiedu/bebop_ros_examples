cmake_minimum_required(VERSION 3.0.2)
project(bebop_gps_example)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  bebop_msgs
	cv_bridge
	image_transport
  roscpp
  sensor_msgs
)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)


catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS bebop_msgs cv_bridge image_transport roscpp sensor_msgs
)


## Specify additional locations of header files
include_directories(include
	include/bebop_gps_example
  ${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)


## Declare a C++ executable
add_executable(bebop_gps_example
	src/bebop_gps_example.cpp
	src/bebop_gps_example_main.cpp
)


## Specify libraries to link a library or executable target against
target_link_libraries(bebop_gps_example
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}	
)
