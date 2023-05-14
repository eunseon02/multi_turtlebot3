#ifndef LIDARKK
#define LIDARKK

#include "lidar.hpp"

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>
#include <geometry_msgs/Point.h>

#include "clusters.hpp"

typedef std::pair<double, double> Point;
typedef std::vector<Point> pointList;

using namespace std;

class Tmo
{
public:
  Tmo();
  ~Tmo();

  void callback(const sensor_msgs::LaserScan::ConstPtr &);
  void Clustering(const sensor_msgs::LaserScan::ConstPtr& , vector<pointList> &);
  void visualiseGroupedPoints(const vector<pointList> &);

  tf::TransformListener tf_listener;
private:

  ros::Subscriber sub_scan;
  ros::Publisher pub_marker_array; 
  vector<Clusters> clusters;

  //Tuning Parameteres
  double dt;
  double euclidean_distance;
  double dth;
  int max_cluster_size;

  laser_geometry::LaserProjection projector_;
  sensor_msgs::LaserScan scan;
  tf::TransformListener listener_;

  unsigned long int cg       = 1;//group counter to be used as id of the clusters
  unsigned long int cclusters= 1;//counter for the cluster objects to be used as id for the markers
  string lidar_frame;
  string world_frame;
};
#endif
