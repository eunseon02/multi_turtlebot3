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


typedef std::pair<double, double> Point;
typedef std::vector<Point> pointList;
laser_geometry::LaserProjection projector_;
visualization_msgs::Marker marker;
visualization_msgs::MarkerArray markera;


void callback(const sensor_msgs::LaserScan::ConstPtr& scan_in){
    sensor_msgs::PointCloud cloud, pc_msg;
    projector_.projectLaser(*scan_in, cloud);
    //marker.pose.pose.position = cloud.points;
    marker.pose.position.x = 1;
    marker.pose.position.y = 1;
    marker.pose.position.z = 1;

    markera.markers.push_back(marker);
}
int main(int argc, char **argv){
    ros::init(argc, argv, "test_node_tt");
    ros::NodeHandle n;
    ros::Subscriber sub_scan = n.subscribe("/tb3_0/scan", 1, callback);
    
    //vector<pointList> point_clusters;
    //visualiseGroupedPoints(point_clusters);
    ros::Publisher pub_marker_array  = n.advertise<visualization_msgs::Marker>("/temp_marker", 10);
	while(ros::ok()){
        marker.header.frame_id = "base_map";
        marker.header.stamp = ros::Time();

        marker.ns = "my_namespace";
        marker.id = 0;

        marker.type = visualization_msgs::Marker::POINTS;
        marker.action = visualization_msgs::Marker::ADD;

        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

    	marker.pose.position.x = 1;
        marker.pose.position.y = 1;
        marker.pose.position.z = 1;

        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;

        markera.markers.push_back(marker);

        pub_marker_array.publish(markera);

        ros::spin();
    }
    return 0;
}