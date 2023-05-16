#include "ros/ros.h"
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"

#include <sstream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_node");
    ros::NodeHandle n;
    //ros::Rate r(30);
    ros::Publisher vis_pub = n.advertise<visualization_msgs::Marker>("/temp_marker", 1);

    //uint32_t shape = visualization_msgs::Marker::CUBE;

    while (ros::ok())
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "base_map";
        marker.header.stamp = ros::Time::now();

        marker.ns = "my_namespace";
        marker.id = 0;

        marker.type = visualization_msgs::Marker::CUBE;
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

        // while (marker_pub.getNumSubscribers() < 1)
        // { 
        //     if (!ros::ok())
        //     {
        //         return 0;
        //     }
        //     ROS_WARN_ONCE("Please create a subscriber to thr marker");
        //     sleep(1);
        // }

        vis_pub.publish(marker);
        ROS_INFO("now");

        //     switch (shape)
        //     {
        //     case visualization_msgs::Marker::CUBE:
        //         shape = visualization_msgs::Marker::SPHERE;
        //         break;
        //     case visualization_msgs::Marker::SPHERE:
        //         shape = visualization_msgs::Marker::ARROW;
        //         break;
        //     case visualization_msgs::Marker::ARROW:
        //         shape = visualization_msgs::Marker::CYLINDER;
        //         break;
        //     case visualization_msgs::Marker::CYLINDER:
        //         shape = visualization_msgs::Marker::CUBE;
        //         break;
        // }

    //r.sleep();
    }
}