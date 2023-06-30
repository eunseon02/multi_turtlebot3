#ifndef TRACKER
#define TRACKER

#pragma once

#include "tracker.hpp"
#include "kalman-cpp/kalman.hpp"
#include <Eigen/Dense>
#include <ros/console.h>

using namespace Eigen;

typedef std::pair<double, double> Point;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

const double pi = 3.141592653589793238463; 

class Tracker {
public:
  
  Tracker(const double& x, const double& y, const double& dt);
  Tracker();
  void update(const double& x, const double& y, const double& dt, const int cluster_size);

private:
  int current_size;
  double test1, test2, test3;
  double x_old, y_old;

  KalmanFilter dynamic_kf;
};
#endif
