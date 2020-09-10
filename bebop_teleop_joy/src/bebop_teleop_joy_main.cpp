#include <bebop_teleop_joy.h>

int main(int argc, char **argv){

    ros::init(argc, argv, "bebop_telelop_joy_node");
    ros::NodeHandle nh;

    BebopJoy bebopJoy(nh);

    ros::spin();

    return 0;
}
