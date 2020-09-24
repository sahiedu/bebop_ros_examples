#include <bebop_gps_example.h>

int main(int argc, char **argv){

    ros::init(argc, argv, "bebop_gps_example_node");
    ros::NodeHandle nh;

    BebopGPS bebopGPS(nh);      // object instance

    ros::spin();

    return 0;

}
