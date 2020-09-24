#ifndef BEBOP_CONTROL_INPUTS_H_
#define BEBOP_CONTROL_INPUTS_H_

// main ROS header
#include <ros/ros.h>

// generically libraries
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

// ROS-OpenCV interface libraries
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// OpenCV libraries
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// ROS messages
#include <geometry_msgs/Twist.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <bebop_msgs/CommonCommonStateBatteryStateChanged.h>

// class definition
class BebopIn{

private:

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber img_sub_;

    // subscribers
    ros::Subscriber twist_sub_;
    ros::Subscriber batt_sub_;

    // publishers
    ros::Publisher ctrl_pub_;

    // image objects
    cv::Mat img_color_;
    cv::Mat img_gray_;
    cv::Mat img_gray_rgb_;

    // ros message variables
    mav_msgs::RollPitchYawrateThrust ctrl_msg_;

    // scaling parameters
    double max_tilt_angle_;
    double max_vert_speed_;
    double max_rot_speed_;

    // control input variables
    double roll_;
    double pitch_;
    double yaw_rate_;
    double thrust_z_;

    // battery state
    int battery_percentage_;
    bool got_first_batt_msg_;


public:

    BebopIn(ros::NodeHandle &nh);       // constructor
    ~BebopIn();                         // destructor

    // member methods
    void control_inputs_publisher();

    // utility methods
    void loadParameters();
    double deg2rad(double deg_angle);
    double rad2deg(double rad_angle);
    std::string num2string(double var);

    // callback functions
    void imageCallback(const sensor_msgs::ImageConstPtr &img_msg);
    void twistCallback(const geometry_msgs::Twist &twist_msg);
    void batteryStateCallback(const bebop_msgs::CommonCommonStateBatteryStateChanged &batt_msg);

};  // end of class definition



#endif
