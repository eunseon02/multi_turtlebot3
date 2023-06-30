#ifndef POINT
#define POINT

// #include "point.hpp"

#pragma once

#include <ros/ros.h>
#include <vector>
#include <cmath>

typedef std::pair<double, double> Point;
typedef std::vector<Point> pointList;
typedef std::vector<Point>::iterator PointIterator;

double length(const Point& p) { return sqrt(pow(p.first, 2.0) + pow(p.second, 2.0)); };
Point operator+ (const Point& p1, const Point& p2) { return Point(p1.first + p2.first, p1.second + p2.second); }
Point operator- (const Point& p1, const Point& p2) { return Point(p1.first - p2.first, p1.second - p2.second); }
Point operator* (const double f, const Point& p)  { return Point(f * p.first, f * p.second); }
// Point operator* (const Point& p, const double f)  { return Point(f * p.first, f * p.second); }
Point operator/ (const Point& p, const double f)  { return (f != 0.0) ? Point(p.first / f, p.second / f) : Point(); }

#endif