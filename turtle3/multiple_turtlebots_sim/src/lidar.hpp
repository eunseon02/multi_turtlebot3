#ifndef LIDARKK
#define LIDARKK

#include "clusters.hpp"
#include "lidar.hpp"

// #include "segment.hpp"

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <vector>

#include <tf/transform_listener.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <laser_geometry/laser_geometry.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>

#include <multiple_turtlebots_sim/TrackArray.h>
#include <multiple_turtlebots_sim/Track.h>
// #include <multiple_turtlebots_sim/Obstacles.h>
// #include <multiple_turtlebots_sim/SegmentObstacle.h>

// #include <armadillo>

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
  // void publishObstacles();
  // Segment fitSegment(const std::vector<Clusters>& clusters); 
  // void detectSegments(const Clusters& cluster);
  // double length(const Point& p) const { return sqrt(pow(p.first, 2.0) + pow(p.second, 2.0)); };

  tf::TransformListener tf_listener;

private:
  ros::Subscriber sub_scan;
  ros::Publisher pub_marker_array; 
  ros::Publisher clustering_res;
  ros::Publisher laser_callback;
  ros::Publisher pub_tracks_circle;
  // ros::Publisher obstacles_pub_;
  // ros::Publisher vis_pub;

  vector<Clusters> clusters;
  // vector<Segment> segments_;

  //Tuning Parameteres
  double dt;
  double euclidean_distance;
  double dth;
  int max_cluster_size;
  bool p_use_split_and_merge_;
  double p_max_group_distance_;
  double p_distance_proportion_;
  double p_max_split_distance_;
  int p_min_group_points_;  
  
  std::vector<Point> input_points_;


  laser_geometry::LaserProjection projector_;
  sensor_msgs::LaserScan scan;

  unsigned long int cg       = 1;//group counter to be used as id of the clusters
  unsigned long int cclusters= 1;//counter for the cluster objects to be used as id for the markers
  string lidar_frame;
  string world_frame;
  bool p_marker_pub;
  
};
#endif
