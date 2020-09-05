#include <bebop_odometry_example.h>

// contructor ------------------------------------------------------------------
BebopOdometry::BebopOdometry(ros::NodeHandle &nh): nh_(nh){

    ROS_INFO("Instance of BebopOdometry class instantiated");

    // odometry subscriber
    odom_sub_ = nh_.subscribe("/bebop/odom", 1, &BebopOdometry::odometryCallback, this);
    // path publisher
    path_pub_ = nh_.advertise<nav_msgs::Path>("/bebop/path", 1);

}


// destructor ------------------------------------------------------------------
BebopOdometry::~BebopOdometry(){
    std::cout << "Instance of BebopOdometry terminated" << std::endl;
}


// odometry callback function
void BebopOdometry::odometryCallback(const nav_msgs::Odometry &odom_msg){
    // data of Bebop's positon will be available after take off

    // populate pose stamped message
    pose_stamped_msg_.header.seq = 0;
    pose_stamped_msg_.header.stamp = ros::Time::now();
    pose_stamped_msg_.header.frame_id = odom_msg.child_frame_id;
    pose_stamped_msg_.pose = odom_msg.pose.pose;

    // populate path message
    path_msg_.header = odom_msg.header;
    path_msg_.poses.push_back(pose_stamped_msg_);

    path_pub_.publish(path_msg_);
}
