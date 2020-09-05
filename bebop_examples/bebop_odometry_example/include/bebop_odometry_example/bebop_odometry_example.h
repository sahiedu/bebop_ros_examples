#ifndef BEBOP_ODOMETRY_EXAMPLE_H_
#define BEBOP_ODOMETRY_EXAMPLE_H_

// main ROS header
#include <ros/ros.h>

// generically libraries
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>


// ROS messages
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>


class BebopOdometry{

private:
    ros::NodeHandle nh_;

    ros::Subscriber odom_sub_;      // subscriber
    ros::Publisher path_pub_;       // publisher

    nav_msgs::Path path_msg_;
    geometry_msgs::PoseStamped pose_stamped_msg_;



public:

    // constructor/destructor
    BebopOdometry(ros::NodeHandle &nh);
    ~BebopOdometry();


    // callback function
    void odometryCallback(const nav_msgs::Odometry &odom_msg);


};  // end of class definition



#endif
