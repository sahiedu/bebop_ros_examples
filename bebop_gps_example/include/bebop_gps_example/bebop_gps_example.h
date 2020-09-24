#ifndef BEBOP_GPS_EXAMPLE_H_
#define BEBOP_GPS_EXAMPLE_H_

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
#include <sensor_msgs/NavSatFix.h>
#include <bebop_msgs/CommonCommonStateBatteryStateChanged.h>


// class definition
class BebopGPS{

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber img_sub_;

    // subscribers
    ros::Subscriber gps_sub_;
    ros::Subscriber batt_sub_;

    // image objects
    cv::Mat img_color_;
    cv::Mat img_gray_;
    cv::Mat img_gray_rgb_;

    // GPS data
    double latitude_;       // Latitude
    double longitude_;      // Longitude
    double altitude_;       // Altitude

    // battery state
    int battery_percentage_;

    bool got_first_batt_msg_;

public:

    BebopGPS(ros::NodeHandle &nh);      // constructor
    ~BebopGPS();                        // destructor

    // utility functions
    std::string num2string(double var);

    // helper functions
    void initSubscribers();

    // callback functions
    void imageCallback(const sensor_msgs::ImageConstPtr &img_msg);
    void gpsCallback(const sensor_msgs::NavSatFix &gps_msg);
    void batteryStateCallback(const bebop_msgs::CommonCommonStateBatteryStateChanged &batt_msg);

};      // end of class definition

#endif
