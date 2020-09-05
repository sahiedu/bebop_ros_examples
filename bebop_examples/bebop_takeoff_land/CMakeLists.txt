cmake_minimum_required(VERSION 3.0.2)
project(bebop_takeoff_land)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
)


## catkin specific configuration
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES bebop_takeoff_land
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs
#  DEPENDS system_lib
)



## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
)


## Declare a C++ executable
add_executable(bebop_takeoff_land
	src/bebop_takeoff_land.cpp
)


## Specify libraries to link a library or executable target against
target_link_libraries(bebop_takeoff_land
	${catkin_LIBRARIES}
)
