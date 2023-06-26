#include "ros/ros.h"
#include "lidar.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar");
    ROS_INFO("main");  
    //Tmo t;
    
    cout << "succsessssss";


    ros::spin();
    return 0;
}
