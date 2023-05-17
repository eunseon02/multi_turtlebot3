#include "ros/ros.h"
#include "lidar.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar");
    
    Tmo t;
    
    cout << "succsessssss";


    ros::spin();
    return 0;
}
