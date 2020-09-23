cmake_minimum_required(VERSION 3.0.2)
project(bebop_odometry_example)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
	geometry_msgs
  	nav_msgs
  	roscpp
)


catkin_package(
	INCLUDE_DIRS include
	#  LIBRARIES bebop_odometry_example
	CATKIN_DEPENDS geometry_msgs nav_msgs roscpp
	#  DEPENDS system_lib
)


## Specify additional locations of header files
include_directories(include
	include/bebop_odometry_example
  	${catkin_INCLUDE_DIRS}
)


## Declare a C++ executable
add_executable(bebop_odometry_example
	src/bebop_odometry_example.cpp
	src/bebop_odometry_example_main.cpp
)


## Specify libraries to link a library or executable target against
target_link_libraries(bebop_odometry_example
	${catkin_LIBRARIES}
)
