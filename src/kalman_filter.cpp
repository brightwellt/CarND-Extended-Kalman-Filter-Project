#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  // lidar and radar use the same function
  x_ = F_ * x_; // lesson 5, 8
  MatrixXd Ft = F_.transpose(); // lesson 5, 9
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
	lesson 5, 7
  */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations 5.14
  */
  float x1 = x_(0);
  float y1 = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  
  // Avoid zero here
  float rho = sqrt(x1 * x1 + y1 * y1);
  float theta = 0.0;
  if (fabs(x1) > 0.0001) {
	  theta = atan2(y1, x1);
  }
  float ro_dot = 0.0;
  if (fabs(rho) > 0.0001) {
	  ro_dot = (x1 * vx + y1 * vy)/rho;
  }
  VectorXd z_pred = VectorXd(3);
  z_pred << rho,theta, ro_dot;
  
  VectorXd y = z - z_pred;
  y[1] = atan2(sin(y[1]), cos(y[1]));
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
  
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

}
