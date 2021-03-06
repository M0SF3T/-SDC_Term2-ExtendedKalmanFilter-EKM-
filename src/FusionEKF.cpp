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
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);

  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */

  //H matrix
  H_laser_ << 1, 0, 0, 0,
			 0, 1, 0, 0;

  //State transition matrix F
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
	  0, 1, 0, 1,
	  0, 0, 1, 0,
	  0, 0, 0, 1;

  //Covariance matrix P
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
	  0, 1, 0, 0,
	  0, 0, 1000, 0,
	  0, 0, 0, 1000;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
		float x_pol_to_car, y_pol_to_car;
		float theta_ = measurement_pack.raw_measurements_[1];


		x_pol_to_car = measurement_pack.raw_measurements_[0] * cos(theta_);
		x_pol_to_car = measurement_pack.raw_measurements_[0] * sin(theta_);

		ekf_.x_ << x_pol_to_car, x_pol_to_car, 1, 1;

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
		ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 1, 1;
    }

	previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  float noise_ax = 9;
  float noise_ay = 9;

  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  cout << "Current timestamp" << measurement_pack.timestamp_ << endl;
  cout << "dt =" << dt << endl;
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  ekf_.Q_ = MatrixXd(4, 4);

  ekf_.Q_ << noise_ax * (dt*dt*dt*dt) / 4, 0, noise_ax * (dt*dt*dt) / 2, 0,
	  0, noise_ay * (dt*dt*dt*dt) / 4, 0, noise_ay * (dt*dt*dt) / 2,
	  noise_ax * (dt*dt*dt) / 2, 0, noise_ax * (dt*dt), 0,
	  0, noise_ay * (dt*dt*dt) / 2, 0, noise_ax * (dt*dt);

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates

	  VectorXd x_state(4);
	  x_state << ekf_.x_(0), ekf_.x_(1), ekf_.x_(2), ekf_.x_(3);

	  //Declaring R and assigning R matrix for radar
	  ekf_.R_ = MatrixXd(3, 3);
	  ekf_.R_ = R_radar_;


	  ekf_.H_ = MatrixXd(3, 4);
	  ekf_.H_ = tools.CalculateJacobian(x_state);

	  ekf_.UpdateEKF(measurement_pack.raw_measurements_);
	  
  } else {
    // Laser updates

	  //Declaring H and R matrixed for Laser
	  ekf_.H_ = MatrixXd(2, 4);
	  ekf_.R_ = MatrixXd(2, 2);

	  //Assigining the values to H and R matrices
	  ekf_.H_ = H_laser_;
	  ekf_.R_ = R_laser_;

	  ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;

}