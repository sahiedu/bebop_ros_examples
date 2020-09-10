#include <bebop_teleop_joy.h>

// constructor -----------------------------------------------------------------
BebopJoy::BebopJoy(ros::NodeHandle &nh): nh_(nh) {

    ROS_INFO("Instance of BebopJoy class instantiated");

    initSubscribers();      // initialize subscribers
    initPublishers();       // initialize publishers
    loadParameters();       // load parameters from parameter server

    is_flying_ = false;
    got_first_joy_msg_ = false;

    // initialize twist message all components to zero
    twist_.linear.x = 0.0;
    twist_.linear.y = 0.0;
    twist_.linear.z = 0.0;
    twist_.angular.x = 0.0;
    twist_.angular.y = 0.0;
    twist_.angular.z = 0.0;

}

// destructor ------------------------------------------------------------------
BebopJoy::~BebopJoy(){
    std::cout << "Instance of BebopJoy terminated" << std::endl;
}

// function that initializes subscribers ---------------------------------------
void BebopJoy::initSubscribers(){
    ROS_INFO("Initializing Subscribers");
    joy_sub_ = nh_.subscribe("/joy", 1, &BebopJoy::joyCallback, this);
    ROS_INFO("Done!");
}

// function that initializes publishers ----------------------------------------
void BebopJoy::initPublishers(){
    ROS_INFO("Initializing Publishers");
    takeoff_pub_ = nh_.advertise<std_msgs::Empty>("/bebop/takeoff", 1);
    land_pub_ = nh_.advertise<std_msgs::Empty>("/bebop/land", 1);
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/bebop/cmd_vel", 1);
    ROS_INFO("Done!");
}


// function that loads parameters from parameter server ------------------------
void BebopJoy::loadParameters(){
    ROS_INFO("Getting parameters from Parameter Server");

    // piloting parameters
    nh_.getParam("/teleop/deadman_button", deadman_button_);
    nh_.getParam("/teleop/piloting/forward_backward/axis", axis_fb_);
    nh_.getParam("/teleop/piloting/forward_backward/scale", scale_fb_);
    nh_.getParam("/teleop/piloting/left_right/axis", axis_lr_);
    nh_.getParam("/teleop/piloting/left_right/scale", scale_lr_);
    nh_.getParam("/teleop/piloting/up_down/axis", axis_ud_);
    nh_.getParam("/teleop/piloting/up_down/scale", scale_ud_);
    nh_.getParam("/teleop/piloting/yawleft_yawright/axis", axis_ylyr_);
    nh_.getParam("/teleop/piloting/yawleft_yawright/scale", scale_ylyr_);
    nh_.getParam("/teleop/takeoff_button", takeoff_button_);
    nh_.getParam("/teleop/land_button", land_button_);

    ROS_INFO("Done!");
}


// joy callback function -------------------------------------------------------
void BebopJoy::joyCallback(const sensor_msgs::Joy &joy_msg){

    if(!got_first_joy_msg_){
        ROS_INFO("Found joystick with %zu Axes and %zu Buttons", joy_msg.axes.size(), joy_msg.buttons.size());

        if(joy_msg.axes.size() == 6 && joy_msg.buttons.size() == 12){
            ROS_INFO("Detected Logitech F710 gamepad in [D] mode");

            std::cout <<    "-------------------------------------------------------------------\n"
                            "Keep pressed the deadman button 'LT' in order to perform any action\n\n"
                            "Y button    ->    Take off\n"
                            "A button    ->    Land\n\n"
                            "Right thumb stick (up/down)    ->  Forward / Backward\n"
                            "Right thumb stick (left/right) ->  Left / Right\n\n"
                            "Left thumb stick (up/down)     ->  Up / Down\n"
                            "Left thumb stick (left/right)  ->  Yaw Left / Yaw Right\n"
                            "-------------------------------------------------------------------" << std::endl;

        }
        else if(joy_msg.axes.size() == 8 && joy_msg.buttons.size() == 11){
            ROS_INFO("Detected Logitech F710 gamepad in [X] mode");
            ROS_WARN("Change to [D] mode and re-run program");
        }
        else{
            ROS_ERROR("Gamepad not supported");
        }

        got_first_joy_msg_ = true;

    }


    // Reading buttons
    bool deadman_button_pressed = joy_msg.buttons.at(deadman_button_);
    bool takeoff_button_pressed = joy_msg.buttons.at(takeoff_button_);
    bool land_button_pressed = joy_msg.buttons.at(land_button_);


    if(deadman_button_pressed){

        // PILOTING
        twist_.linear.x = scale_fb_ * joy_msg.axes[axis_fb_];       // Right thumb stick (up/down) -> forward / backward
        twist_.linear.y = scale_lr_ * joy_msg.axes[axis_lr_];       // Right thumb stick (left/right) -> left / right
        twist_.linear.z = scale_ud_ * joy_msg.axes[axis_ud_];       // Left thumb stick (up/down) -> up / down
        twist_.angular.z = scale_ylyr_ * joy_msg.axes[axis_ylyr_];  // Left thumb stick (left/right) -> yaw left / yaw right


        // TAKEOFF
        if(!is_flying_ && takeoff_button_pressed){
            //ROS_INFO("Taking off!");
            takeoff_pub_.publish(std_msgs::Empty());
            is_flying_ = true;
        }

        // LAND
        if(is_flying_ && land_button_pressed){
            //ROS_INFO("Landing");
            land_pub_.publish(std_msgs::Empty());
            is_flying_ = false;
        }

    }

    publish_cmd();

}


// functions that publish twist commands ---------------------------------------
void BebopJoy::publish_cmd(){
    vel_pub_.publish(twist_);
}
