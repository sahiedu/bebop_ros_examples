#include <bebop_control_inputs.h>

int main(int argc, char **argv){

    ros::init(argc, argv, "bebop_control_inputs_node");
    ros::NodeHandle nh;

    BebopIn bebopIn(nh);        // object instance
    ros::Rate loop_rate(50);    // publishing rate in Hz

    while(ros::ok()){
        bebopIn.control_inputs_publisher();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
