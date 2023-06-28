#ifndef LIDARKK
#define LIDARKK

#include "lidar.hpp"
#include "clusters.hpp"

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
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>

#include <multiple_turtlebots_sim/TrackArray.h>
#include <multiple_turtlebots_sim/Track.h>

typedef std::pair<double, double> Point;
typedef std::vector<Point> pointList;

using namespace std;

class Tmo
{
public:
  Tmo();
  ~Tmo();

  void callback(const sensor_msgs::LaserScan::ConstPtr &);
  void Clustering(vector<pointList> &, vector< vector<float> >& ,const int );
  void visualizeGroupedPoints(const std::vector<pointList>& point_clusters);
  void LiDARmsg(const sensor_msgs::LaserScan::ConstPtr &);

  tf::TransformListener tf_listener;

private:
  ros::Subscriber sub_scan;
  ros::Publisher pub_marker_array; 
  ros::Publisher clustering_res;
  ros::Publisher laser_callback;
  // ros::Publisher vis_pub;
  vector<Clusters> clusters;
  vector<pointList> point_clusters;

  //Tuning Parameteres
  double dt;
  double euclidean_distance;
  double dth;
  int max_cluster_size;

  laser_geometry::LaserProjection projector_;
  sensor_msgs::LaserScan scan;

  unsigned long int cg       = 1;//group counter to be used as id of the clusters
  unsigned long int cclusters= 1;//counter for the cluster objects to be used as id for the markers
  string lidar_frame;
  string world_frame;
  bool p_marker_pub;
  
};
#endif
