#include <bebop_control_inputs.h>

// constructor -----------------------------------------------------------------
BebopIn::BebopIn(ros::NodeHandle &nh): it_(nh_) {

    ROS_INFO("Instance of BebopIn class instantiated");

    // load parameters from parameter server
    loadParameters();

    // initialize subscribers
    img_sub_ = it_.subscribe("/bebop/image_raw", 1, &BebopIn::imageCallback, this);
    twist_sub_ = nh_.subscribe("/bebop/cmd_vel", 1, &BebopIn::twistCallback, this);
    batt_sub_ = nh_.subscribe("/bebop/states/common/CommonState/BatteryStateChanged", 1, &BebopIn::batteryStateCallback, this);

    // initialize publisher
    ctrl_pub_ = nh_.advertise<mav_msgs::RollPitchYawrateThrust>("/bebop/cmd_ctrl", 1);

    // initialize RollPitchYawrateThrust message
    ctrl_msg_.roll = 0.0;
    ctrl_msg_.pitch = 0.0;
    ctrl_msg_.yaw_rate = 0.0;
    ctrl_msg_.thrust.x = 0.0;
    ctrl_msg_.thrust.y = 0.0;
    ctrl_msg_.thrust.z = 0.0;

    // initialize control variables
    roll_ = 0.0;
    pitch_ = 0.0;
    yaw_rate_ = 0.0;
    thrust_z_ = 0.0;

    // battery state initial values
    battery_percentage_ = 0;
    got_first_batt_msg_ = false;

}


// destructor ------------------------------------------------------------------
BebopIn::~BebopIn(){
    cv::destroyAllWindows();
    std::cout << "Instance of BebopIn terminated" << std::endl;
}


// function that loads parameters from parameter server ------------------------
void BebopIn::loadParameters(){
    ROS_INFO("Getting parameters from Parameter Server");

    // scaling parameters
    nh_.param("/bebop/bebop_driver/PilotingSettingsMaxTiltCurrent", max_tilt_angle_, 20.0);
    nh_.param("/bebop/bebop_driver/SpeedSettingsMaxVerticalSpeedCurrent", max_vert_speed_, 1.0);
    nh_.param("/bebop/bebop_driver/SpeedSettingsMaxRotationSpeedCurrent", max_rot_speed_, 100.0);

    ROS_INFO("Done!");
}


// convert from radians to degrees ---------------------------------------------
double BebopIn::deg2rad(double deg_angle){
    double rad_angle = deg_angle * M_PI / 180.0;
    return rad_angle;
}


// convert from degrees to radians ---------------------------------------------
double BebopIn::rad2deg(double rad_angle){
    double deg_angle = rad_angle * 180.0 / M_PI;
    return deg_angle;
}


// converts number datatype to string ------------------------------------------
std::string BebopIn::num2string(double var){
    std::ostringstream str_var;
    str_var << var;
    return str_var.str();
}


// twist callback function -----------------------------------------------------
void BebopIn::twistCallback(const geometry_msgs::Twist &twist_msg){

    // compute control inputs
    roll_ = deg2rad(twist_msg.linear.y * max_tilt_angle_);
    pitch_ = deg2rad(twist_msg.linear.x * max_tilt_angle_);
    yaw_rate_ = deg2rad(twist_msg.angular.z * max_rot_speed_);
    thrust_z_ = twist_msg.linear.z * max_vert_speed_;

    // populate RollPitchYawrateThrust message
    ctrl_msg_.header.seq = 0;
    ctrl_msg_.header.stamp = ros::Time::now();
    ctrl_msg_.header.frame_id = ' ';

    ctrl_msg_.roll = roll_;             // Roll angle [rad], roll rotates around new x-axis
    ctrl_msg_.pitch = pitch_;           // Pitch angle [rad], pitch rotates around new y-axis
    ctrl_msg_.yaw_rate = yaw_rate_;     // Yaw rate around z-axis [rad/s], yaw rotates around fixed frame's z axis
    ctrl_msg_.thrust.z = thrust_z_;     // Thrust [m/s] expressed in the body frame, the z-component is used. Set all un-used components to 0.
                                        // To convert to [N] first estimate the acceleration and then apply the formula F = m*a, where m = 0.5 kg

}


// Image callback function -----------------------------------------------------
void BebopIn::imageCallback(const sensor_msgs::ImageConstPtr &img_msg){

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


    // put input data on the screen
    cv::putText(img_gray_rgb_, "roll: " + num2string(rad2deg(roll_)) + " [deg]", cv::Point(5, 15), CV_FONT_HERSHEY_COMPLEX_SMALL, 0.7, cv::Scalar(0, 0, 255), 1, 1);
    cv::putText(img_gray_rgb_, "pitch: " + num2string(rad2deg(pitch_)) + " [deg]", cv::Point(5, 30), CV_FONT_HERSHEY_COMPLEX_SMALL, 0.7, cv::Scalar(0, 0, 255), 1, 1);
    cv::putText(img_gray_rgb_, "yaw rate: " + num2string(rad2deg(yaw_rate_)) + " [deg/sec]", cv::Point(5, 45), CV_FONT_HERSHEY_COMPLEX_SMALL, 0.7, cv::Scalar(0, 0, 255), 1, 1);
    cv::putText(img_gray_rgb_, "thrust z: " + num2string(thrust_z_) + " [m/sec]", cv::Point(5, 60), CV_FONT_HERSHEY_COMPLEX_SMALL, 0.7, cv::Scalar(0, 0, 255), 1, 1);


    // put battery state on the screen
    if(got_first_batt_msg_){
        cv::putText(img_gray_rgb_, "Battery percentage: " + num2string(battery_percentage_) + " [%]", cv::Point(5, 470), CV_FONT_HERSHEY_COMPLEX_SMALL, 0.7, cv::Scalar(255, 0, 255), 1, 1);
    }
    else{
        cv::putText(img_gray_rgb_, "Battery percentage: -- [%]", cv::Point(5, 470), CV_FONT_HERSHEY_COMPLEX_SMALL, 0.7, cv::Scalar(255, 0, 255), 1, 1);
    }


    cv::imshow("Bebop Camera", img_gray_rgb_);
    cv::waitKey(1);

}


// function that publishes the control inputs ----------------------------------
void BebopIn::control_inputs_publisher(){
    // publish message
    ctrl_pub_.publish(ctrl_msg_);
}


// battery state callback function ---------------------------------------------
void BebopIn::batteryStateCallback(const bebop_msgs::CommonCommonStateBatteryStateChanged &batt_msg){

    if(!got_first_batt_msg_){

        got_first_batt_msg_ = true;
    }

    // Battery percentage
    battery_percentage_ = batt_msg.percent;

}
