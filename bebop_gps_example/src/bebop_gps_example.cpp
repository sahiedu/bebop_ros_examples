#include <bebop_gps_example.h>

// constructor -----------------------------------------------------------------
BebopGPS::BebopGPS(ros::NodeHandle &nh): it_(nh_){

    ROS_INFO("Instance of BebopGPS class instantiated");

    // initialize subscribers
    initSubscribers();

    // initialize member variables
    latitude_ = 0.0;
    longitude_ = 0.0;
    altitude_ = 0.0;

    battery_percentage_ = 0;

    got_first_batt_msg_ = false;
}


// destructor ------------------------------------------------------------------
BebopGPS::~BebopGPS(){
    cv::destroyAllWindows();
    std::cout << "Instance of BebopGPS terminated" << std::endl;
}


// function that initializes subscribers
void BebopGPS::initSubscribers(){
    ROS_INFO("Initializing Subscribers");
    img_sub_ = it_.subscribe("/bebop/image_raw", 1, &BebopGPS::imageCallback, this);
    gps_sub_ = nh_.subscribe("/bebop/fix", 1, &BebopGPS::gpsCallback, this);
    batt_sub_ = nh_.subscribe("/bebop/states/common/CommonState/BatteryStateChanged", 1, &BebopGPS::batteryStateCallback, this);
    ROS_INFO("Done!");
}


// converts number datatype to string ------------------------------------------
std::string BebopGPS::num2string(double var){
    std::ostringstream str_var;
    str_var << var;
    return str_var.str();
}


// Image callback function -----------------------------------------------------
void BebopGPS::imageCallback(const sensor_msgs::ImageConstPtr &img_msg){

    cv_bridge::CvImagePtr cv_ptr;       // OpenCV data type

    try{
        // conversion from ROS message to OpenCV object
        cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception &e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    img_color_ = cv_ptr->image;

    // convert color image to grayscale
    cv::cvtColor(img_color_, img_gray_, CV_BGR2GRAY);
    cv::cvtColor(img_gray_, img_gray_rgb_, CV_GRAY2BGR);


    // put GPS data on the screen
    cv::putText(img_gray_rgb_, "Latitude: " + num2string(latitude_) + " [deg]", cv::Point(5, 15), CV_FONT_HERSHEY_COMPLEX_SMALL, 0.7, cv::Scalar(0, 0, 255), 1, 1);
    cv::putText(img_gray_rgb_, "Longitude: " + num2string(longitude_) + " [deg]", cv::Point(5, 30), CV_FONT_HERSHEY_COMPLEX_SMALL, 0.7, cv::Scalar(0, 0, 255), 1, 1);
    cv::putText(img_gray_rgb_, "Altitude: " + num2string(altitude_) + " [m]", cv::Point(5, 45), CV_FONT_HERSHEY_COMPLEX_SMALL, 0.7, cv::Scalar(0, 0, 255), 1, 1);

    // put battery state on the screen
    if(got_first_batt_msg_){
        cv::putText(img_gray_rgb_, "Battery percentage: " + num2string(battery_percentage_) + " [%]", cv::Point(5, 85), CV_FONT_HERSHEY_COMPLEX_SMALL, 0.7, cv::Scalar(255, 0, 255), 1, 1);
    }
    else{
        cv::putText(img_gray_rgb_, "Battery percentage: -- [%]", cv::Point(5, 85), CV_FONT_HERSHEY_COMPLEX_SMALL, 0.7, cv::Scalar(255, 0, 255), 1, 1);
    }


    cv::imshow("Bebop Camera", img_gray_rgb_);
    cv::waitKey(1);

}


// GPS callback function -------------------------------------------------------
void BebopGPS::gpsCallback(const sensor_msgs::NavSatFix &gps_msg){
    // store GPS data
    // data will be available once the bebop is in the air
    latitude_ = gps_msg.latitude;
    longitude_ = gps_msg.longitude;
    altitude_ = gps_msg.altitude;
}


// battery state callback function ---------------------------------------------
void BebopGPS::batteryStateCallback(const bebop_msgs::CommonCommonStateBatteryStateChanged &batt_msg){

    if(!got_first_batt_msg_){

        got_first_batt_msg_ = true;
    }

    // Battery percentage
    battery_percentage_ = batt_msg.percent;

}
