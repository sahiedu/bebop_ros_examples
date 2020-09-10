# bebop_examples

The **bebop_examples** collection features a variety of packages to help ROS users get the Parrot Bebop drone flying quickly. This code has been tested on the the Bebop 2. These nodes rely on the [bebop_autonomy](https://github.com/AutonomyLab/bebop_autonomy) package which acts as the driver for the robot.

## Dependencies
This examples depends on the following external packages:

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
This program performs a simple takeoff, keeps the drone in hover mode for about 5 seconds, and then lands.
