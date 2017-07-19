#include "kalman_filter.h"

// may remove this include later
#include <iostream>

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

  //-------------------------------------------------------------------------------
  //
  // code entered June-10-2017
  //
  //-------------------------------------------------------------------------------

  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;

 return;

}


void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */

  //-------------------------------------------------------------------------------
  //
  // code entered June-10-2017
  //
  //-------------------------------------------------------------------------------

  VectorXd z_pred = H_ * x_;
  VectorXd y   = z - z_pred;
  MatrixXd Ht  = H_.transpose();
  MatrixXd S   = H_ * P_ * Ht + R_;
  MatrixXd K   = P_ * Ht * S.inverse();

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

  return;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

  //-------------------------------------------------------------------------------
  //
  // code entered June-10-2017
  //
  //-------------------------------------------------------------------------------

  float ph    = x_(0)*x_(0) + x_(1)*x_(1);

  // check for division by zero
  if (ph < 0.001) {
    std::cout << "ERROR (UpdateEKF): division by zero !" << std::endl;
    return;
  }

  float p    =  sqrt(ph);
  float a    =  atan2(x_(1),x_(0));    
  float pdot =  (x_(0)*x_(2) + x_(1)*x_(3))/p;

 
  // Vector h
  VectorXd h(3); 
           h << p, a, pdot;

  VectorXd y = z - h;
 
  NORM:; 
  // normalize a if needed
  if (y(1) > M_PI ) {
    //std::cout << "NOTE (UpdateEKF): angle not normalized ! value = " << y(1) << std::endl;
    y(1) -= 2 * M_PI;
    //std::cout << "NOTE (UpdateEKF): angle corrected      ! value = " << y(1) << std::endl;
    goto NORM;
  } else if (y(1) < -M_PI) { 
    //std::cout << "NOTE (UpdateEKF): angle not normalized ! value = " << y(1) << std::endl;
    y(1) += 2 * M_PI;
    //std::cout << "NOTE (UpdateEKF): angle corrected      ! value = " << y(1) << std::endl;
    goto NORM;
  } 

  MatrixXd Ht  = H_.transpose();
  MatrixXd S   = H_ * P_ * Ht + R_;
  MatrixXd K   = P_ * Ht * S.inverse();

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  
  //std::cout << "NOTE (UpdateE): before P_ " << std::endl << P_ << std::endl;
  P_ = (I - K * H_) * P_;

  return;

  
}      
