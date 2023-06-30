#include "tracker.hpp"

Tracker::Tracker(){}//Creates a blank estimator

Tracker::Tracker(const double& x, const double& y, const double& dt){

  // Initialization of Dynamic Kalman Filter
  int n = 6; // Number of states
  int m = 2; // Number of measurements
  MatrixXd A(n, n); // System dynamics matrix
  MatrixXd C(m, n); // Output matrix
  MatrixXd Q(n, n); // Process noise covariance
  MatrixXd R(m, m); // Measurement noise covariance
  MatrixXd P(n, n); // Estimate error covariance
      
  //A << 1, 0,dt, 0, 
       //0, 1, 0,dt, 
       //0, 0, 1, 0, 
       //0, 0, 0, 1;
       //
  double ddt = dt*dt/2;
  A << 1, 0,dt, 0, ddt,   0, 
       0, 1, 0,dt,   0, ddt, 
       0, 0, 1, 0,  dt,   0, 
       0, 0, 0, 1,   0,  dt,
       0, 0, 0, 0,   1,   0, 
       0, 0, 0, 0,   0,   1;

  C << 1, 0, 0, 0, 0, 0,
       0, 1, 0, 0, 0, 0;

  Q << 1, 0, 0, 0, 0, 0,
       0, 1, 0, 0, 0, 0,
       0, 0,10, 0, 0, 0,
       0, 0, 0,10, 0, 0,
       0, 0, 0, 0,20, 0,
       0, 0, 0, 0, 0,20;
  R.setIdentity();
  R *=10;
  R *= 0.1;
  P.setIdentity() * 0.1;

  KalmanFilter dynamic_kalman_filter(dt, A, C, Q, R, P); 
  this->dynamic_kf = dynamic_kalman_filter;

  VectorXd x0_dynamic(n);
  x0_dynamic << x, y, 0, 0, 0, 0;  
  dynamic_kf.init(0,x0_dynamic);

  x_old = x;
  y_old = y;

}

void Tracker::update(const double& x, const double& y, const double& dt, const int cluster_size) {

  current_size = cluster_size;
  
  // Update Dynamic Kalman Filter
  Vector2d z;
  z << x, y;
  dynamic_kf.update(z, dt);

  x_old = x;
  y_old = y;

}
