#include "kalman_filter.h"
#include "tools.h"

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
	x_ = F_ * x_;
	// Declaring and calculatin transpose matrix of F
	MatrixXd F_t_ = F_.transpose();

	P_ = F_ * P_ * F_t_ + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
	//Setting up the variable requirements
	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	// Calculating the new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
	//recover state parameters
	float px = x_(0);
	float py = x_(1);
	float vx = x_(2);
	float vy = x_(3);

	VectorXd x_state(4);
	x_state << px, py, vx, vy;


	//Compute the values of rho, theta and ro dot

	float rho = sqrt((px*px) + (py*py));
	float theta = atan2(py, px);
	float ro_dot = (px*vx + py*vy) / rho;


	//Declaring a prediction vector with the calculated polar parameters
	VectorXd z_pred = VectorXd(3);
	z_pred << rho, theta, ro_dot;


	//VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;

	while (y[1] < -M_PI)
		y[1] += 2 * M_PI;
	while (y[1] > M_PI)
		y[1] -= 2 * M_PI;

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
