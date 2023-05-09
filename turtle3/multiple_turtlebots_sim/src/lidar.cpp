#include "lidar.hpp"

Tmo::Tmo(){
    ros::NodeHandle n;
    sub_scan = n.subscribe("/scan", 1, &Tmo::callback, this);
}

Tmo::~Tmo(){
  
}

void Tmo::callback(const sensor_msgs::LaserScan::ConstPtr& scan_in){
    if(!listener_.waitForTransform(
            scan_in->header.frame_id,
            "/base_link",
            scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
            ros::Duration(1.0))){
        return;
    }

    sensor_msgs::PointCloud cloud;
    projector_.transformLaserScanToPointCloud("/base_link",*scan_in,
            cloud,listener_);

    vector<pointList> point_clusters;
    //Tmo::Clustering(scan_in, point_clusters);
}

void Tmo::Clustering(const sensor_msgs::LaserScan::ConstPtr& scan_in, vector<pointList> &clusters){
    
}

