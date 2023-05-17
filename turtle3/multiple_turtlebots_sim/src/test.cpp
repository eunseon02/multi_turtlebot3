// #include <ros/ros.h>
// #include <tf/tf.h>
// #include <geometry_msgs/Twist.h>
// #include <sensor_msgs/LaserScan.h>
// #include <sensor_msgs/PointCloud.h>
// #include <laser_geometry/laser_geometry.h>
// #include <vector>
// #include <geometry_msgs/Point32.h>
// #include <visualization_msgs/Marker.h>
// #include <visualization_msgs/MarkerArray.h>
// #include <cmath>

// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_types.h>

// #include <pcl/conversions.h>

// typedef std::pair<double, double> Point;
// typedef std::vector<Point> pointList;
// laser_geometry::LaserProjection projector_;
// visualization_msgs::Marker marker;
// visualization_msgs::MarkerArray markera;
// //ros::Publisher pub_marker_array  = n.advertise<visualization_msgs::Marker>("/temp_marker", 10);
// ros::NodeHandle n;
// ros::Publisher pub_marker_array  = n.advertise<visualization_msgs::Marker>("/temp_marker", 10);

// void callback(const sensor_msgs::LaserScan::ConstPtr& scan_in){
//     sensor_msgs::PointCloud cloud;
//     projector_.projectLaser(*scan_in, cloud);
    
//     pcl::PointCloud<pcl::PointXYZ> cloud_dst;
//     pcl::fromROSMsg(cloud, cloud_dst);
//     for(unsigned int i=0;i<cloud_dst.size();i++){
//         geometry_msgs::Point p;
//         p.x = cloud_dst.points[i].x;
//         p.y = cloud_dst.points[i].y;
//         p.z = cloud_dst.points[i].z;
//         marker.points.push_back(p);
//     }
//     //markera.markers.push_back(marker);
//     pub_marker_array.publish(marker);
// }

// int main(int argc, char **argv){
//     ros::init(argc, argv, "points_marker_node");
//     // ros::NodeHandle n;
//     // ros::Publisher pub_marker_array  = n.advertise<visualization_msgs::Marker>("/temp_marker", 10);
//     ros::Subscriber sub_scan = n.subscribe("/tb3_0/scan", 1, callback);
    

//     ros::Rate r(30);
//     //ros::Publisher pub_marker_array  = n.advertise<visualization_msgs::Marker>("/temp_marker", 10);

//     float f = 0.0;
// 	while(ros::ok()){
//     marker.header.frame_id = "base_map";
//     marker.header.stamp = ros::Time::now();

//     marker.ns = "my_namespace";
//     marker.pose.orientation.w = 1.0;
//     marker.id = 0;

//     marker.type = visualization_msgs::Marker::POINTS;
//     marker.action = visualization_msgs::Marker::ADD;

//     marker.color.r = 0.0f;
//     marker.color.g = 1.0f;
//     marker.color.b = 0.0f;
//     marker.color.a = 1.0;

//     marker.pose.position.x = 1;
//     marker.pose.position.y = 1;
//     marker.pose.position.z = 1;

//     marker.scale.x = 1.0;
//     marker.scale.y = 1.0;

//     ///pub_marker_array.publish(marker);
//     r.sleep();

//     f += 0.04;
//     }
// }