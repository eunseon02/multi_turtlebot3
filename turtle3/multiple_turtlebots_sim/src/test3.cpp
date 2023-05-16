#include "ros/ros.h"
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"

#include <sstream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_node");
    ros::NodeHandle n;
    ros::Publisher vis_pub = n.advertise<visualization_msgs::Marker>("/tmp_marker", 1);


    while (ros::ok())
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "my_map";
        marker.header.stamp = ros::Time::now();

        marker.ns = "my_namespace";
        marker.id = 0;

        marker.type = visualization_msgs::Marker::POINTS;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;

        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;

        marker.color.a = 1.0; 
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;

        marker.lifetime = ros::Duration();

        vis_pub.publish(marker);
        ROS_INFO("now");

    }
}