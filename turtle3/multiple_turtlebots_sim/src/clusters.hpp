#ifndef CLUSTERSS
#define CLUSTERSS

#include"clusters.hpp"

#pragma once
#include <ros/ros.h>
#include <vector>

typedef std::pair<double, double> Point;
typedef std::vector<Point> pointList;

class Clusters {
public:    
    Clusters(unsigned long int id, const pointList&);

    unsigned long int id; //identifier for the cluster 
    float r, g, b, a; //color of the cluster

    //void update(const pointList&);

    std::pair<double, double> mean() { return mean_values; }; //Return mean of cluster.
    double meanX() { return mean_values.first; };
    double meanY() { return mean_values.second;};

private:
  std::pair<double, double> mean_values;
};

#endif