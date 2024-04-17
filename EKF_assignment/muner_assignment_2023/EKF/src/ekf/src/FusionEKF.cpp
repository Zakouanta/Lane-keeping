#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF()
{
	is_initialized_ = false;

	previous_timestamp_ = 0;

	// initializing matrices
	R_laser_ = MatrixXd(2, 2);
	R_radar_ = MatrixXd(3, 3);
	H_laser_ = MatrixXd(2, 4);
	Hj_ = MatrixXd(3, 4);

	// measurement covariance matrix - laser.

	R_laser_ << 0.0225, 0,
		0, 0.0225;

	// measurement covariance matrix - radar
	R_radar_ << 0.09, 0, 0,
		0, 0.0009, 0,
		0, 0, 0.09;

	// measurement matrix
	// H_laser_ = MatrixXd(2, 4); I commented this because it has been already initialised
	H_laser_ << 1, 0, 0, 0,
		0, 1, 0, 0;

	// Jacobian Matrix is the derivative of h(x) with respect to x, it is a matrix containing all the partial derivatives
	Hj_ << 1, 1, 0, 0,
		1, 1, 0, 0,
		1, 1, 1, 1;

	/**
	 * Finish initializing the FusionEKF.
	 * Set the process and measurement noises
	 */
	// create a 4D state vector, we don't know yet the values of the x state
	//ekf_.x_ = VectorXd(4);

	/** TODO
	 * Initialize the state matrix P
	 * here we set the values of P matrix such that the error is not dragged in time.
	 * Change here the values to 1000.0 to set set a big enough value to not drag the error
	 */
	ekf_.P_ = MatrixXd(4, 4);
	ekf_.P_ << 1000.0, 0, 0, 0,
		0, 1000.0, 0, 0,
		0, 0, 1000.0, 0,
		0, 0, 0, 1000.0;

	// the initial transition matrix F_
	ekf_.F_ = MatrixXd(4, 4);
	ekf_.F_ << 1, 0, 1, 0,
		0, 1, 0, 1,
		0, 0, 1, 0,
		0, 0, 0, 1;
	noise_ax = 9;
	noise_ay = 9;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack)
{

	/*****************************************************************************
	 *  Initialization
	 ****************************************************************************/
	if (!is_initialized_)
	{

		/**
		 * Initialize the state ekf_.x_ with the first measurement.
		 * Create the covariance matrix.
		 * Remember: you'll need to convert radar from polar to cartesian coordinates.
		 */
		// first measurement
		cout << "EKF: " << endl;
		ekf_.x_ = VectorXd(4);

		// initialisation
		//ekf_.x_ << 1, 1, 1, 1;

		previous_timestamp_ = measurement_pack.timestamp_;
		if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
		{
			/**
			Convert radar from polar to cartesian coordinates and initialize state.
			*/
			float rho = measurement_pack.raw_measurements_[0];
			float phi = measurement_pack.raw_measurements_[1];
			float rho_dot = measurement_pack.raw_measurements_[2];
			ekf_.x_ << rho * cos(phi), rho * sin(phi), rho_dot * cos(phi), rho_dot * sin(phi);
		}
		else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
		{
			/**
			Initialize state.
			*/
			ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
		}
		// done initializing, no need to predict or update
		is_initialized_ = true;
		return;
	}

	/*****************************************************************************
	 *  Prediction
	 ****************************************************************************/
	/**
	 * Update the state transition matrix F according to the new elapsed time.
		- Time is measured in seconds.
	*/
	// compute the time elapsed between the current and previous measurements
	float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0; // dt - expressed in seconds
	previous_timestamp_ = measurement_pack.timestamp_;

	/** TODO
	 * Modify the F matrix so that the time is integrated
	 */
	float dt_2 = dt * dt;
	float dt_3 = dt_2 * dt;
	float dt_4 = dt_3 * dt;

	// We modify the output such that the position components (px and py)
	// are updated based on the velocity components (vx and vy) and the time step (dt)
	ekf_.F_ << 1, 0, dt, 0,
		0, 1, 0, dt,
		0, 0, 1, 0,
		0, 0, 0, 1;

	/** TODO
	 * set the process covariance matrix Q
	 * use noise_ax = 9 and noise_ay = 9 for your Q matrix.
	 */

	// To do so we need to define first the variable of our noise as parameters of the process covariance matrix. Here we increase and decrease the noise to observe what happens 
	
	float noise_ax = 9;  // 2, 20, 100 
	float noise_ay = 9;  //   2, 20, 100 
	ekf_.Q_ = MatrixXd(4, 4);
	// Based on the matrix structure of the Q matrix we have
	ekf_.Q_ << dt_4/4*noise_ax, 0, dt_3/2*noise_ax , 0,
		0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
		dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
		0, dt_3/2*noise_ay, 0, dt_2*noise_ay;
	ekf_.Predict();

	/*****************************************************************************
	 *  Update
	 ****************************************************************************/

	/**
	 * Use the sensor type to perform the update step.
	 * Update the state and covariance matrices.
	 */

	// initialise the matrix H

	if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
	{
		Tools t_;
		/** TODO
		 * Calculate the jacobian and call the UpdateEKF function
		 */
		// Calculate the Jacobian of the  H matrix
		ekf_.H_ = t_.CalculateJacobian(ekf_.x_);
		ekf_.R_ = R_radar_;
		// call the updateEKF function
		ekf_.Init(ekf_.x_, ekf_.P_, ekf_.F_, ekf_.H_, ekf_.R_, ekf_.Q_);
		ekf_.UpdateEKF(measurement_pack.raw_measurements_);
	}
	else
	{
		ekf_.H_ = H_laser_;
		ekf_.R_ = R_laser_;
		/** TODO
		 * Calculate the update function
		 */
		// call the update function

		ekf_.Init(ekf_.x_, ekf_.P_, ekf_.F_, ekf_.H_, ekf_.R_, ekf_.Q_);
		ekf_.Update(measurement_pack.raw_measurements_);
	}

	// print the  output
	cout << "x_ = " << ekf_.x_ << endl;
	cout << "P_ = " << ekf_.P_ << endl;
}
