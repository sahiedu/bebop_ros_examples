#include <bebop_teleop_joy.h>

int main(int argc, char **argv){

    ros::init(argc, argv, "bebop_telelop_joy_node");
    ros::NodeHandle nh;

    BebopJoy bebopJoy(nh);      // object instance
    ros::Rate loop_rate(50);    // publishing rate in Hz

    while(ros::ok()){
        bebopJoy.publish_cmd();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
