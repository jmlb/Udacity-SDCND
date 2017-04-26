#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;
//define a threshold value
#define epsilon 0.00001
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
        //Process covariance matrix
  MatrixXd Q_ = MatrixXd(4, 4);


  /* Create a 4D state vector and initialize to 1*/
  VectorXd x_ = VectorXd(4);
  x_ << epsilon, epsilon,epsilon,epsilon;

  //measurement matrix
  H_laser_ << 1, 0, 0, 0,
            0, 1, 0, 0;

  //the initial transition matrix F_
  MatrixXd F_ = MatrixXd(4, 4);
  F_ << 1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;

  /* Initialize State Covariance matrix */
  MatrixXd P_ = MatrixXd(4, 4);
  P_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1000, 0,
        0, 0, 0, 1000;

  ekf_.Init(x_, P_, F_, H_laser_, R_laser_, Q_);

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
    // first measurement
    //cout << "EKF: " << endl;
    //Time stamp
    previous_timestamp_ = measurement_pack.timestamp_;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      float ro = measurement_pack.raw_measurements_[0];
      float phi = measurement_pack.raw_measurements_[1];
      float ro_dot =  measurement_pack.raw_measurements_[2];
      //Convert radial to cartesian coordinates
      float px = ro * cos(phi);
      float py = ro * sin(phi);
      float vx = ro_dot * cos(phi);
      float vy = ro_dot * sin(phi);
      // set state vector
      ekf_.x_ << px, py, vx, vy;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      float px = measurement_pack.raw_measurements_[0];
      float py = measurement_pack.raw_measurements_[1];
      float vx = 0;
      float vy = 0;
      // set state vector
      ekf_.x_ << px, py, vx, vy;
    }

    
    
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  double sigma2_ax = 9.0;
  double sigma2_ay = 9.0;
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
  //compute dt time
  float dt = (measurement_pack.timestamp_ - previous_timestamp_)/1000000.0;
  // set current time stamp to previous
  previous_timestamp_ = measurement_pack.timestamp_;
  //update transition matrix
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;
  //Process Covariance Matrix
  //set the process covariance matrix
  float dt2 = dt * dt;
  float dt3 = dt2 * dt;
  ekf_.Q_ << dt3*dt/4 * sigma2_ax, 0, dt2*dt/2 * sigma2_ay, 0,
        0, dt3*dt/4 *sigma2_ay, 0, dt2*dt/2*sigma2_ay,
        dt2*dt/2*sigma2_ax, 0, dt2*sigma2_ax, 0,
        0, dt2*dt/2 * sigma2_ay, 0, dt2*sigma2_ay;
  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }
  
  // print the output
  //cout << "x_ = " << ekf_.x_ << endl;
  //cout << "P_ = " << ekf_.P_ << endl;
}