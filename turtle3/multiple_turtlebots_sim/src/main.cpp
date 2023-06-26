#include "ros/ros.h"
#include "lidar.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar");

    Tmo t;

    ros::spin();
    return 0;
}
