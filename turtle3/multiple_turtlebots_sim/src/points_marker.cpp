#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <laser_geometry/laser_geometry.h>
#include <vector>
#include <geometry_msgs/Point32.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <cmath>


typedef std::pair<double, double> Point;
typedef std::vector<Point> pointList;
laser_geometry::LaserProjection projector_;
visualization_msgs::Marker marker;
visualization_msgs::MarkerArray markera;

// void callback(const sensor_msgs::LaserScan::ConstPtr& scan_in){
//     sensor_msgs::PointCloud cloud, pc_msg;
//     projector_.projectLaser(*scan_in, cloud);
//     marker.pose.position.x = 1;
//     marker.pose.position.y = 1;
//     marker.pose.position.z = 1;

//     markera.markers.push_back(marker);
// }

int main(int argc, char **argv){
    ros::init(argc, argv, "points_marker_node");
    ros::NodeHandle n;
    // ros::Subscriber sub_scan = n.subscribe("/tb3_0/scan", 1, callback);
    

    ros::Rate r(30);
    ros::Publisher pub_marker_array  = n.advertise<visualization_msgs::Marker>("/temp_marker", 10);

    float f = 0.0;
	while(ros::ok()){
    marker.header.frame_id = "base_map";
    marker.header.stamp = ros::Time::now();

    marker.ns = "my_namespace";
    marker.pose.orientation.w = 1.0;
    marker.id = 0;

    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.pose.position.x = 1;
    marker.pose.position.y = 1;
    marker.pose.position.z = 1;

    marker.scale.x = 1.0;
    marker.scale.y = 1.0;


    // Create the vertices for the points and lines
    for (uint32_t i = 0; i < 100; ++i)
    {
        float y = 5 * sin(f + i / 100.0f * 2 * M_PI);
        float z = 5 * cos(f + i / 100.0f * 2 * M_PI);

        geometry_msgs::Point p;
        p.x = (int32_t)i - 50;
        p.y = y;
        p.z = z;

        marker.points.push_back(p);

        p.z += 1.0;

    }

    pub_marker_array.publish(marker);
    r.sleep();

    f += 0.04;
    }
}