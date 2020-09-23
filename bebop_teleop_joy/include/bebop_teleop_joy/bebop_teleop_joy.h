#ifndef BEBOP_TELEOP_JOY_H_
#define BEBOP_TELEOP_JOY_H_

// main ROS header
#include <ros/ros.h>

// generically libraries
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

// ROS messages
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Empty.h>

// class definition
class BebopJoy{

private:
    ros::NodeHandle nh_;

    // subscriber
    ros::Subscriber joy_sub_;

    // publishers
    ros::Publisher takeoff_pub_;
    ros::Publisher land_pub_;
    ros::Publisher vel_pub_;

    geometry_msgs::Twist twist_;

    // flags
    bool got_first_joy_msg_;
    bool is_flying_;

    // piloting parameters
    int deadman_button_;

    int axis_fb_;
    int axis_lr_;
    int axis_ud_;
    int axis_ylyr_;
    double scale_fb_;
    double scale_lr_;
    double scale_ud_;
    double scale_ylyr_;

    int takeoff_button_;
    int land_button_;

public:

    BebopJoy(ros::NodeHandle &nh);      // constructor
    ~BebopJoy();                        // destructor

    // command publisher
    void publish_cmd();

    // helper functions
    void initSubscribers();
    void initPublishers();
    void loadParameters();

    // joy callback function
    void joyCallback(const sensor_msgs::Joy &joy_msg);

};      // end of class definition

#endif
