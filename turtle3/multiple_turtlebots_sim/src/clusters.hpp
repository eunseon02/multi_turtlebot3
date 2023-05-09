#ifndef CLUSTERSS
#define CLUSTERSS

#include"clusters.hpp"

#include <ros/ros.h>
#include <vector>

class Clusters {
public:    
    Cluster(unsigned long int id, const pointList&);

    int id; //identifier for clusters

    void update(const pointList&);
};

#endif