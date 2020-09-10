# bebop_ros_examples

This collection features a variety of packages to help ROS users get the Parrot Bebop drone flying quickly. These examples rely on the [bebop_autonomy](https://github.com/AutonomyLab/bebop_autonomy) package which acts as the driver for the robot. The entire repository should be cloned to: ```~/catkin_ws/src```. To do so, navigate to ```~/catkin_ws/src``` and from a terminal and enter: ```git clone https://github.com/EDU4RDO-SH/bebop_ros_examples.git```. Then, navigate to ```~/catkin_ws``` and compile the code with the command: ```catkin_make```. If you are installing ROS for the first time, see the instructions [here](https://wiki.ros.org/kinetic/Installation/Ubuntu). This version has been created using the Bebop 2, ROS Kinetic and Ubuntu 16.04.


<p align="center"><img src="https://i.imgur.com/yY6nKXf.png" width="400" /></p>


## Dependencies
The listed examples depends on the following external packages:

- [bebop_autonomy](https://bebop-autonomy.readthedocs.io/en/latest/#)
- [joy](https://wiki.ros.org/joy)
- [mav_msgs](https://wiki.ros.org/mav_msgs)

## Setup

### 1. Driver installation
A detailed description for the correct installation of the Bebop driver can be found [here](https://bebop-autonomy.readthedocs.io/en/latest/installation.html), in a nutshell, we only need to clone the driver packages and compile the code.

### 2. Clone the repository
Next, the entire repository should be cloned to: ```~/catkin_ws/src``` by entering the command ```git clone https://github.com/EDU4RDO-SH/ROS.git``` and compile the code with ```catkin_make```.


## Examples

### bebop_takeoff_land
This program performs a simple takeoff, keeps the drone in hover mode 1 meter above the takeoff point for a few seconds, and then lands. First, run the driver:

```
roslaunch bebop_driver bebop_node.launch
```

Then execute the takeoff and land program:

```
roslaunch bebop_takeoff_land bebop_takeoff_land.launch
```

### bebop_teleop_joy
This package allows us to control the Bebop drone with the ```Logitech F710``` gamepad. This program depends on the [joy](https://wiki.ros.org/joy) which enables the use of the gamepad with ROS.



### bebop_odometry_example
This program subscribes to the ```/bebop/odom``` topic data published by the Bebop and populate a ```nav_msgs/Path``` message for further visualization in Rviz.


<p align="center"><img src="https://i.imgur.com/G9SAn9K.png" width="1000" /></p>
