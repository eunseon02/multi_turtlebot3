#ifndef SEGMENT
#define SEGMENT

#pragma once

// #include "segment.hpp"
#include "clusters.hpp"
// #include "point.hpp"
// #include "lidar.hpp"

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
//----point관련 
  double dot(const Point& p1, const Point& p2)   const { return p1.first * p2.first + p1.second * p2.second; };
  double lengthSquared(const Point& p) const { return pow(p.first, 2.0) + pow(p.second, 2.0); };
  double length(const Point& p)const { return sqrt(pow(p.first, 2.0) + pow(p.second, 2.0)); };
//--
  double cross(const Point& p1 , const Point& p2) const { 
    return p1.first * p2.second - p1.second * p2.first; };

  double distanceTo(const Point& p) const {
    Point tmp = Point(p.first - projection(p).first, p.second - projection(p).second);
    return length(tmp);
  };

  Point projection(const Point& p) const {
    Point a = Point(last_point.first - first_point.first, last_point.second - first_point.second);
    Point b = Point(p.first - first_point.first, p.second - first_point.second);
    // Point b = p - first_point;
    // return first_point + dot(a, b) * a / lengthSquared(a);
    Point tmp1 = Point(dot(a, b) * a.first, dot(a, b) * a.second);
    Point tmp2 = Point(first_point.first + tmp1.first, first_point.second + tmp1.second);
    return Point(tmp2.first / lengthSquared(a), tmp2.second / lengthSquared(a));
  };
  
  Point first_point;
  Point last_point;
  std::vector<Clusters> clusters;
};

#endif