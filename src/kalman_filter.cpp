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
  std::cout << "hi" << std::endl;
  F_ = MatrixXd(4, 4);
  Q_ = MatrixXd(4, 4);
  x_ = MatrixXd(4, 1);
  P_ = MatrixXd(4, 4); // Object Covariance Matrix
  I_ = MatrixXd(4, 4); 
  H_ = MatrixXd(2, 4);
  //Todo: Is this right???
  P_ << 100, 0, 0, 0,
      0, 100, 0, 0,
      0, 0, 10, 0,
      0, 0, 10, 0;

  F_ << 1, 0, 1, 0,
      0, 1, 0, 1,
      0, 0, 1, 0,
      0, 0, 0, 1;

  x_ << 0, 0, 0, 0;

      
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
  VectorXd x_prime = F_ * x_;
  MatrixXd p_prime = F_ * P_ * F_.transpose() + Q_;

  x_ = x_prime;
  P_ = p_prime;

}

void printShape(MatrixXd &mat, std::string name) {
  std::cout << name;
  std::cout << "rows: " << mat.rows() << " cols: " << mat.cols() << std::endl;
}
void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  VectorXd y = z - H_ * x_; 
  MatrixXd S = (H_ * P_ * H_.transpose()) + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();

  VectorXd x_prime = x_ + (K * y);

  MatrixXd IKH = (I_ - K * H_);
  MatrixXd P_prime = IKH * P_;

  x_ = x_prime;
  P_ = P_prime;

}

// What's the point of using this ??? is this just for radar measurments?
void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */

  // Translate x_ into range data
  float x_val = x_[0];
  float y_val = x_[1];
  float v_x = x_[2];
  float v_y = x_[3];

  float rho = sqrt(pow(x_val, 2) + pow(x_val, 2));
  float phi = atan2(y_val, x_val);
  float rho_dot;
  if (abs(rho) < 0.01) {
    rho_dot = 0;
  } else {
    rho_dot = ((x_val * v_x) + (y_val * v_y)) / rho;
  }

  VectorXd range_pred(3);
  range_pred << rho, phi, rho_dot;


  VectorXd y = z - range_pred;
  MatrixXd S = (H_ * P_ * H_.transpose()) + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();

  VectorXd x_prime = x_ + (K * y);

  MatrixXd IKH = (I_ - K * H_);
  MatrixXd P_prime = IKH * P_;

  x_ = x_prime;
  P_ = P_prime;

}
