cmake_minimum_required(VERSION 2.8.3)
project(bebop_teleop_joy)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
	geometry_msgs	
	joy
  roscpp
	sensor_msgs
	std_msgs
)


catkin_package()


## Specify additional locations of header files
include_directories(include
	include/bebop_teleop_joy
  ${catkin_INCLUDE_DIRS}
)


## Declare a C++ executable
add_executable(bebop_teleop_joy
	src/bebop_teleop_joy.cpp
	src/bebop_teleop_joy_main.cpp
)


## Specify libraries to link a library or executable target against
target_link_libraries(bebop_teleop_joy
	${catkin_LIBRARIES}	
)
