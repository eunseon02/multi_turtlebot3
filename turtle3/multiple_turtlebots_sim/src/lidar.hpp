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
#include <vector>
#include <geometry_msgs/Point.h>

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
private:
  ros::Subscriber sub_scan;
  laser_geometry::LaserProjection projector_;
  tf::TransformListener listener_;
};
#endif
