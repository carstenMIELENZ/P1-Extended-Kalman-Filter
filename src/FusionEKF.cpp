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
  Hj_      = MatrixXd(3, 4);

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
 
  //-------------------------------------------------------------------------------
  //
  // code entered June-10-2017
  //
  //-------------------------------------------------------------------------------
 
  // set H matrix for later
  H_laser_ <<  1, 0, 0, 0,
               0, 1, 0, 0;  
 
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 0.15, 0, 0, 0,
    	     0, 0.15, 0, 0,
	     0, 0, 10, 0,
	     0, 0, 0, 10; 	
  //Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
  //                      MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) 

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
    cout << "init EKF ..." << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    //-------------------------------------------------------------------------------
    //
    // code entered June-10-2017
    //
    //-------------------------------------------------------------------------------

    // set start time
    previous_timestamp_ = measurement_pack.timestamp_; 

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
   
      // dont think that you need to convert p. here since v = p / dt but dt is unknown   
      float px = measurement_pack.raw_measurements_[0] * sin(measurement_pack.raw_measurements_[1]);
      float py = measurement_pack.raw_measurements_[0] * cos(measurement_pack.raw_measurements_[1]);
        
      ekf_.x_ << px, py, 0, 0;   
      
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
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
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

   //-------------------------------------------------------------------------------
   //
   // code entered June-10-2017
   //
   //-------------------------------------------------------------------------------
 
   //compute the time elapsed between the current and previous measurements
   float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	
    // set previous time 
    previous_timestamp_ = measurement_pack.timestamp_; 
	
   // Modify the F matrix so that the time is integrated
   ekf_.F_ = MatrixXd(4, 4); 
   ekf_.F_ << 1, 0, dt, 0,
   	      0, 1, 0, dt,
	      0, 0, 1, 0,
	      0, 0, 0, 1;

   // Set the process covariance matrix Q
   const float NOISE_AX = 9;
   const float NOISE_AY = 9;

   float dt2 = dt  * dt;
   float dt3 = dt2 * dt;
   float dt4 = dt3 * dt;
   
   float q33 = NOISE_AX * dt2;
   float q44 = NOISE_AY * dt2;
   float q11 = q33 * dt2 / 4;
   float q22 = q44 * dt2 / 4;
   float q13 = dt3 * NOISE_AX / 2;
   float q24 = dt3 * NOISE_AY / 2;

   ekf_.Q_ = MatrixXd(4, 4);
   ekf_.Q_ << q11, 0, q13, 0,
              0, q22, 0, q24,
              q13, 0, q33, 0,
              0, q24, 0, q44;

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
      Hj_     = tools.CalculateJacobian(ekf_.x_);
      ekf_.H_ = Hj_;
      ekf_.R_ = R_radar_;
      ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
      // Laser updates
      ekf_.H_ = H_laser_;
      ekf_.R_ = R_laser_;
      ekf_.Update(measurement_pack.raw_measurements_);
  }

}

