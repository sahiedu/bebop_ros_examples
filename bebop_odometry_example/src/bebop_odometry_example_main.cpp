#include <bebop_odometry_example.h>

int main(int argc, char **argv){

    ros::init(argc, argv, "bebop_odometry_example_node");
    ros::NodeHandle nh;

    BebopOdometry bebopOdometry(nh);

    ros::spin();

    return 0;
    
}
