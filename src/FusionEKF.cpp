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
  H_laser_ << 1, 0, 0, 0, // Lesson 5, 10
              0, 1, 0, 0;

			  
  //ekf_.F_ = // 4x4 matrix (state transition) - done in ProcessMeasurement
  // ekf_.P_ = // 4x4 matrix // https://discussions.udacity.com/t/rmse-for-x-starts-high-and-does-not-dip-to-0-11-by-the-end-of-the-data-feed/309561
  ekf_.P_ = MatrixXd(4,4);
  ekf_.P_ << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 100, 0,
          0, 0, 0, 100;
			  
  //set the acceleration noise components
  //noise_ax = 9; // lesson 5 section 13
  //noise_ay = 9;


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
    ekf_.x_ << 1, 1, 1, 1; // this value is important for the RMSE - play with the latter two values as we may accumulate high RMSE at the beginning

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
	  We get rho, theta, rho_dot. Convert them over!
      */
	  //Just set ekf_.x_{0} to ro*cos(theta)
	  //Just set ekf_.x_{1} to ro*sin(theta)
	  float ro = measurement_pack.raw_measurements_[0];
	  float theta = measurement_pack.raw_measurements_[1];
	  ekf_.x_ << ro*cos(theta), ro*sin(theta), 0, 0;
	  
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
	  // Copy x and y over.
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
      
    }

	//ekf_.F_ << set to 1 diagonal matrix (4x4, following classroom stuff)
	ekf_.F_ = MatrixXd(4, 4);
	ekf_.F_ << 1, 0, 0, 0,
	           0, 1, 0, 0,
			   0, 0, 1, 0,
			   0, 0, 0, 1;
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
   //compute the time elapsed between the current and previous measurements
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;

  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  //Modify the F matrix so that the time is integrated lesson 5.8
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  float noise_ax = 9; // lesson 5 section 13
  float noise_ay = 9;
  //set the process covariance matrix Q lesson 5.9
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
         0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
         dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
         0, dt_3/2*noise_ay, 0, dt_2*noise_ay;

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
	// Set ekf_.H_ by setting to Hj, the calculated jacobian 
	// set ekf_.R_ by just using R_radar_
	Hj_ = tools.CalculateJacobian(ekf_.x_);
	ekf_.H_= Hj_;
	ekf_.R_ = R_radar_;
	
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
	// Set ekf_.H_ by just using H_laser_
	// Set ekf_.R by just using R_laser
	ekf_.H_ = H_laser_;
	ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
