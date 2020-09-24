cmake_minimum_required(VERSION 3.0.2)
project(bebop_control_inputs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  bebop_msgs
  cv_bridge
  geometry_msgs
  image_transport
  mav_msgs
  roscpp
	sensor_msgs
  std_msgs
)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)


## catkin specific configuration
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS bebop_msgs cv_bridge geometry_msgs image_transport mav_msgs roscpp sensor_msgs std_msgs
)


## Specify additional locations of header files
include_directories(include
	include/bebop_control_inputs
  ${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)


## Declare a C++ executable
add_executable(bebop_control_inputs
	src/bebop_control_inputs.cpp
	src/bebop_control_inputs_main.cpp
)


## Specify libraries to link a library or executable target against
target_link_libraries(bebop_control_inputs
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}	
)
