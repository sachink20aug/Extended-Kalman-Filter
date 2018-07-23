#include "kalman_filter.h"
#include<iostream>
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

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
  
	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
	std::cout<<"Update"<<std::endl;
	VectorXd y=z-H_*x_;
	MatrixXd S = H_ * P_ * H_.transpose() + R_;
	MatrixXd Si = S.inverse();
  
	MatrixXd K = P_*H_.transpose() * Si;
	x_ = x_ + (K * y);
	MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
	P_ = (I - K * H_) * P_;
  
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
	std::cout<<"UpdateEKF"<<std::endl;
	VectorXd h_x = MatrixXd(3, 1);
	const double pi=3.14159;
    bool flag_range = false;
	double rho,phi, rho_dot;
	rho = sqrt((x_[0]*x_[0]) + (x_[1]*x_[1]));
  

    if (rho <= 0.001) {   //if rho is very small
    phi = 0;
    rho_dot = 0;
	} 
	
	else {
    phi = atan2(x_[1] , x_[0]);
    rho_dot = ((x_[0] * x_[2] + x_[1] * x_[3]) / rho);
  }
  
	h_x << rho, phi, rho_dot;
	VectorXd y=z-h_x;
  
    //make sure phi lies between -pi to +pi
    while (flag_range == false) {
      if (y(1) > pi) {
        y(1) = y(1) - pi;
      }
      else if (y(1) < -pi) {
        y(1) = y(1) + pi;
      } 
	  else {
        flag_range = true;
      }
    }
    
	

	MatrixXd S = H_ * P_ * H_.transpose() + R_;
	MatrixXd Si = S.inverse();
  
	MatrixXd K = P_*H_.transpose() * Si;
	x_ = x_ + (K * y);
	MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
	P_ = (I - K * H_) * P_;
}
