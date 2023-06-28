#ifndef CLUSTERSS
#define CLUSTERSS

#include"clusters.hpp"

#pragma once
#include <ros/ros.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "multiple_turtlebots_sim/Track.h"
#include "utils/MAF.hpp"

typedef std::pair<double, double> Point;
typedef std::vector<Point> pointList;

class Clusters {
public:    
    Clusters(unsigned long int id, const pointList&, const double& dt);

    unsigned long int id; //identifier for the cluster 
    float r, g, b, a; //color of the cluster

    multiple_turtlebots_sim::Track msg_track;
    void update(const pointList&, const double& dt);

    std::pair<double, double> mean() { return mean_values; }; //Return mean of cluster.
    double meanX() { return mean_values.first; };
    double meanY() { return mean_values.second;};
    visualization_msgs::Marker getVisualisationMessage();

private:
  std::pair<double, double> mean_values;
  void calcMean(const pointList& ); //Find the mean value of the cluster
  void populateTrackingMsgs(const double& dt);
};

#endif