// main ROS header
#include <ros/ros.h>

// generically libraries
#include <iostream>

// ROS messages
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>

// global variables
geometry_msgs::Twist twist_msg;


// hover function --------------------------------------------------------------
void hover(){
    twist_msg.linear.x = 0.0;
    twist_msg.linear.y = 0.0;
    twist_msg.linear.z = 0.0;
    twist_msg.angular.x = 0.0;
    twist_msg.angular.y = 0.0;
    twist_msg.angular.z = 0.0;

    ROS_INFO("All twist message components set to 0");
}



// main program ----------------------------------------------------------------
int main(int argc, char **argv){

    ros::init(argc, argv, "bebop_takeoff_land_node");
    ros::NodeHandle nh;

    // publishers
    ros::Publisher takeoff_pub = nh.advertise<std_msgs::Empty>("/bebop/takeoff", 1000);
    ros::Publisher land_pub = nh.advertise<std_msgs::Empty>("/bebop/land", 1000);
    ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("/bebop/cmd_vel", 1000);


    // define message objects
    std_msgs::Empty takeoff_msg;
    std_msgs::Empty land_msg;


    // perform simple takeoff and landing

    hover();                        // initialize drone in rest
    ros::Duration(2).sleep();       // delay before start

    ROS_INFO("***** T  A  K  E  O  F  F *****");
    takeoff_pub.publish(takeoff_msg);

    ros::Duration(1).sleep();       // climbing time
    hover();                        // hover mode
    ros::Duration(6).sleep();       // keeps in hover mode during 6 secs

    ROS_INFO("***** L  A  N  D *****");
    land_pub.publish(land_msg);

    ros::spinOnce();

    return 0;
}
