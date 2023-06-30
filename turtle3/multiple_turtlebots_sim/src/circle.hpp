#ifndef CIRCLE
#define CIRCLE

#pragma once

// #include "clusters.hpp"
#include "circle.hpp"

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <vector>

typedef std::pair<double, double> Point;
typedef std::vector<Point> pointList;

class Circle
{
public:
  Circle(const Point& p, const double r = 0.0) : center(p), radius(r) { };

  // Circle(const Cluster& c) {
  //   radius = 0.5773502 * s.length();  // sqrt(3)/3 * length
  //   center = (s.first_point + s.last_point - radius * s.normal()) / 2.0;
  //   clusters = s.clusters;
  // }

  Point center;
  double radius;
  // std::vector<Clusters> clusters;
};

#endif