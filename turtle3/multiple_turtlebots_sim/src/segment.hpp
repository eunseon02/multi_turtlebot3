#ifndef SEGMENT
#define SEGMENT

#pragma once

#include "clusters.hpp"

#include <vector>

typedef std::pair<double, double> Point;
typedef std::vector<Point> pointList;

class Segment
{
public:
  Segment(const Point& p1 , const Point& p2){
    // Swap if not counter-clockwise
    if (cross(p1, p2) > 0.0)
      first_point = p1, last_point = p2;
    else
      first_point = p2, last_point = p1;
  };
  double cross(const Point& p1 , const Point& p2) const { 
    return p1.first * p2.second - p1.second * p2.first; };

  Point first_point;
  Point last_point;
  std::vector<Clusters> clusters;
};

#endif