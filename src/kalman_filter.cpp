#include "kalman_filter.h"
#include <iostream>
using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() 
{
  F_ = MatrixXd(4, 4);
  Q_ = MatrixXd(4, 4);
  x_ = MatrixXd(4, 1);
  P_ = MatrixXd(4, 4); // Object Covariance Matrix
  I_ = MatrixXd(4, 4); 
  H_ = MatrixXd(2, 4);
  //Todo: Is this right???
  P_ << 1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1500, 0,
      0, 0, 0, 1500;

  F_ << 1, 0, 1, 0,
      0, 1, 0, 1,
      0, 0, 1, 0,
      0, 0, 0, 1;


      
}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
  F_ = F_in;
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
   x_ = F_ * x_ ;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;

}

void printShape(MatrixXd &mat, std::string name) {
  std::cout << name;
  std::cout << "rows: " << mat.rows() << " cols: " << mat.cols() << std::endl;
}
void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  VectorXd laser_pred = H_ * x_;
  VectorXd y = z - laser_pred;
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd PHt = P_ * H_.transpose();
  MatrixXd K = PHt * S.inverse();

  //new estimate
  x_ = x_ + (K * y);
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I - K * H_) * P_;

}
double normalizeAngle(double radians) {
  while (radians > M_PI) {
    radians -= M_PI;
  }
  while (radians < -M_PI) {
    radians += M_PI;
  }
  return radians;
}

// What's the point of using this ??? is this just for radar measurments?
void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  float rho = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
  float phi = atan2(x_(1), x_(0));



  float rho_dot;
  if (fabs(rho) < 0.0001) {
    rho_dot = 0;
  } else {
    rho_dot = (x_(0)*x_(2) + x_(1)*x_(3))/rho;
  }
  VectorXd z_pred(3);
  z_pred << rho, phi, rho_dot;
  VectorXd y = z - z_pred;

  //Normalize the angle
  double normalized = normalizeAngle(y(1));
  y(1) = normalized;
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd PHt = P_ * H_.transpose();
  MatrixXd K = PHt * S.inverse();

  //new estimate
  x_ = x_ + (K * y);
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I - K * H_) * P_;

}

