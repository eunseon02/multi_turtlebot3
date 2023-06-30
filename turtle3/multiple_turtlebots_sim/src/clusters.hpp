#ifndef CLUSTERSS
#define CLUSTERSS

#pragma once
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <vector>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "multiple_turtlebots_sim/Track.h"

#include "clusters.hpp"
#include "circle.hpp"
#include "tracker.hpp"

typedef std::pair<double, double> Point;
typedef std::vector<Point> pointList;
typedef std::vector<Point>::iterator PointIterator;

class Clusters {
public:    
  Clusters(unsigned long int id, const pointList& points, const double& dt);

  unsigned long int id; //identifier for the cluster 
  unsigned long int age; //age of the cluster 
  float r, g, b, a; //color of the cluster

  multiple_turtlebots_sim::Track msg_track;

  void update(const pointList&, const double& dt);

  std::pair<double, double> mean() { return mean_values; }; //Return mean of cluster.
  double meanX() { return mean_values.first; };
  double meanY() { return mean_values.second;};

  visualization_msgs::Marker getVisualisationMessage(); 
  visualization_msgs::Marker CircleVisualisationMessage();

  int num_points;
  double p_max_circle_radius_ = 0.5;
  double radius;

  Tracker circle;

  PointIterator begin, end; 
  pointList points;
  std::vector<Circle> circles_;

  double cx, cy, cvx, cvy;

private:
  pointList new_cluster;
  Point center_point;
  Point mean_values;
  Point previous_mean_values;
  
  void populateTrackingMsgs(const double& dt);
  void calcMean(const pointList& ); //Find the mean value of the cluster
  void findFarthestPoints(const std::vector<Point>& cluster, Point& point1, Point& point2);
  void Circlefitting(const pointList& clusters, Point mean_values); 
};

#endif