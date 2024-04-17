#include "kalman_filter.h"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
						MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in)
{
	x_ = x_in;
	P_ = P_in;
	F_ = F_in;
	H_ = H_in;
	R_ = R_in;
	Q_ = Q_in;
}

void KalmanFilter::Predict()
{
	std:: cout << "predict" <<endl;

	/*
	TODO:
	* predict the state
	Here we define the values of the states and the state covariance matrix based on the formulas of the notes 
	*/
	x_ = F_ * x_; // The motion noise is neglected 
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_ ;
}

void KalmanFilter::Update(const VectorXd &z)
{
	std:: cout << "update" <<endl;
	/*
	TODO:
	* update the state by using Kalman Filter equations
	* Here we do the same as in the predict case 
	*/
	VectorXd y = z - (H_*x_); 
	MatrixXd Ht = H_.transpose(); // use the transpose() function
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse() ; // use the inverse() function
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	/*
	TODO:
	* Compute the new estimate
	x = x_ + Ky
	*/
	x_ =  x_ + (K * y ); 
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_ ;
}

void KalmanFilter::UpdateEKF(const VectorXd &z)
{
	std:: cout << "updatedEKF" <<endl;

	/*
	TODO:
	* update the state by using Extended Kalman Filter equations
	* We first need to take the cartesian values and then convert them to Polar one 
	Based on the famous expresion r^2 = x^2 +  y^2 and tan teta = x/y
	*/
    // Initialising the parameters of the state vectors' components 

    float px = x_(0);
	float py = x_(1);
	float vx = x_(2);
    float vy = x_(3);
    
	// Now we convert 

	float pi = atan(1) * 4;

	float rho = sqrt((px * px) + (py * py));
	float phi = atan2 (py, px) ;

	float rho_dot;
	if (fabs(rho) < 0.0001)
	{
		rho_dot = 0;
	}
	else
	{
		// 
		rho_dot = (px * vx + py * vy)/rho ;
	}
	VectorXd z_pred(3);
	z_pred << rho, phi, rho_dot;

	VectorXd y = z - z_pred ;

	if (y[1] >= pi)
	{ // Normalizing Angles
		y[1] = y[1] - (2 * pi);
	}
	else if (y[1] < -pi)
	{
		y[1] = y[1] + (2 * pi);
	}

	MatrixXd Ht = H_.transpose() ; // use the transpose() function
	MatrixXd S = H_ * P_ * Ht + R_; 
	MatrixXd Si = S.inverse(); // use the inverse() function
	MatrixXd PHt = P_ * Ht;
	MatrixXd K =  PHt * Si;

	// new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_)* P_;
}
