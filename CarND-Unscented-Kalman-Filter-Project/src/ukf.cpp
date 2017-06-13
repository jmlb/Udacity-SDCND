#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;
//threshold value
#define epsilon 0.00001
/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  /**
  Complete the initialization. See ukf.h for other member properties.
  */	
  // initially set to false, set to true in first call of ProcessMeasurement
  is_initialized_ = false;
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;
  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // Initialize state dimension
  n_x_ = 5;
  //Initialize time
  time_us_ = 0;
  // Initialize dimension of augmented state vector
  n_aug_ = n_x_ + 2;
  // Number of sigma points
  n_sigma_pts = n_aug_ * 2 + 1;
  //initialize lambda: scaling factor
  lambda_ = 3 - n_aug_;

  // initial state vector - 'dummy initialization'
  x_ = VectorXd(n_x_);
  x_.fill(0.0);
  // initialize state covariance matrix to Identity matrix
  P_ = MatrixXd(n_x_, n_x_);
  MatrixXd I = MatrixXd::Identity(n_x_, n_x_);
  P_ << I * 0.8;
  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 0.8;
  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.5;
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;
  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;
  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;
  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;
  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  
  Xsig_pred_ = MatrixXd(n_x_, n_sigma_pts);
  //Initialize weights
  weights_ = VectorXd(n_sigma_pts);
  //set weight value for each sigma points
  for(int i=0; i<n_sigma_pts; i++){
	  if(i==0){ weights_(i) = lambda_/(lambda_ + n_aug_);}
	  else{ weights_(i) = 0.5 /(lambda_ + n_aug_);}
  }
  //cout<<"weights" << weights_ <<endl;
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /** first measurement */
  if (!is_initialized_) {
    //Get measurement timestamp
    time_us_ = meas_package.timestamp_;

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      //Convert radar data from polar to cartesian coordinates 
      float ro = meas_package.raw_measurements_[0];
      float phi = meas_package.raw_measurements_[1];
      float dro =  meas_package.raw_measurements_[1];
      float px = ro * cos(phi);
      float py = ro * sin(phi);
      float vx = dro * cos(phi);
      float vy = dro * sin(phi);
      float v = sqrt(vx*vx + vy*vy);
      // initialize only positions px, py, v of state vector.
	  x_.head(3) << px, py, v;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      float px = meas_package.raw_measurements_[0];
      float py = meas_package.raw_measurements_[1];
      // Initialize px and py of state vector
      x_.head(2) << px, py;
    }
    // done initializing
    is_initialized_ = true;
    return;
  }
  /** Prediction based on CRTV model */
  float dt = (meas_package.timestamp_ - time_us_)/1000000.0;	//time step dt [sec]
  time_us_ = meas_package.timestamp_; //update time
  // Run prediction using augmented state and sigma points
  Prediction(dt);
  //cout<<"Prediction done"<<endl;
  
  /** Measurement update */
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    UpdateRadar(meas_package);
    //cout<<"Radar Measurement update done "<<endl;
  }
  else {
    UpdateLidar(meas_package);
    //cout<<"Lidar Measurement update done "<<endl;
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /** Augmentation: generate sigma points from current state vector */
  //create augmented state vector
  VectorXd x_aug = VectorXd(n_aug_);
  x_aug.fill(0.0);
  x_aug.head(x_.size()) << x_;  //(last 2 components = 0 - mean value of yaw acceleration and longitudinal acceleration noise = 0)
  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  P_aug.fill(0.0);
  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, n_sigma_pts);
  //create augmented covariance matrix
  P_aug.block(0,0,n_x_,n_x_) << P_;
  P_aug(5,5) = std_a_ * std_a_;
  P_aug(6,6) = std_yawdd_ * std_yawdd_;
  
  //create square root matrix of P_aug
  MatrixXd A = P_aug.llt().matrixL();
  //Generate augmented sigma points
  Xsig_aug.col(0) << x_aug;
  for(int i=1; i<n_aug_+1; i++){
      Xsig_aug.col(i) = x_aug + sqrt(lambda_ + n_aug_)*A.col(i-1);
      Xsig_aug.col(i+n_aug_) = x_aug - sqrt(lambda_ + n_aug_)*A.col(i-1); 
  }
  /** Predict sigma points using CTRV model*/
  for(int i=0; i< n_sigma_pts; i++){
      double px = Xsig_aug(0, i);
      double py = Xsig_aug(1, i);
      double v = Xsig_aug(2, i);
      double psi = Xsig_aug(3, i);
      double psi_rate = Xsig_aug(4, i);
      double nu_a = Xsig_aug(5, i);
      double nu_psi = Xsig_aug(6, i);
      //Deterministic + stochastic part
      if(fabs(psi_rate) < epsilon){
		psi_rate = 0;
		Xsig_pred_(0,i) = px +  v * cos(psi) * delta_t + 0.5 * delta_t * delta_t * cos(psi) * nu_a;
        Xsig_pred_(1,i) = py + v * sin(psi) * delta_t + 0.5 * delta_t * delta_t * sin(psi) * nu_a;
        Xsig_pred_(3,i) = psi +  0 + 0.5 * delta_t * delta_t * nu_psi;
      }
      else{
        Xsig_pred_(0,i) =  px + v/psi_rate * (sin(psi + psi_rate*delta_t) - sin(psi)) + 0.5 * delta_t * delta_t * cos(psi) * nu_a;
        Xsig_pred_(1,i) =  py + v/psi_rate * (-cos(psi + psi_rate*delta_t) +cos(psi)) + 0.5 * delta_t * delta_t * sin(psi) * nu_a;
        Xsig_pred_(3,i) =  psi + psi_rate * delta_t + 0.5 * delta_t * delta_t * nu_psi;
      }
      
      Xsig_pred_(2,i) =  v + 0 + delta_t * nu_a;
      Xsig_pred_(4,i) =  psi_rate + 0 + delta_t * nu_psi;
  }
  //cout<<"predicted sigma Xsig_pred_"<< Xsig_pred_<<endl;
  /** Compute predicted mean state and covariance matrix from sigma points */
  //predicted state mean
  x_ << Xsig_pred_ * weights_; // Xsig_pred * weights_ = (5*15)*(15*1) = (5*1) vector
  //predicted state covariance matrix
  MatrixXd x_diff =  (Xsig_pred_).colwise() - x_ ; //compute x_diff (sigma_pt - mean)
  //Keep yaw angle in range [-PI, PI]
  for(int i=0; i<n_sigma_pts; i++){
    while (x_diff(3,i)> M_PI) x_diff(3,i)-=2.*M_PI;
    while (x_diff(3,i)<-M_PI) x_diff(3,i)+=2.*M_PI;
  }
  MatrixXd weighted_dist =  (x_diff).array().rowwise() * weights_.array().transpose(); // compute weighted delta
  P_ << ( weighted_dist * x_diff.transpose());
  //cout<<"predicted sigma P_"<< P_<<endl;
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /** Convert sigma points from process model space to measurement model space */
  //set dimension of measurement vector radar_meas = (px, py)
  int n_z = 2;
  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, n_sigma_pts);
  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  //transform sigma points into measurement space
  for(int i=0; i< (2*n_aug_ + 1); i++){
      Zsig(0, i) = Xsig_pred_(0,i);
      Zsig(1, i) =  Xsig_pred_(1,i);
  }

  //calculate mean predicted measurement
  z_pred = Zsig * weights_;
  //calculate measurement covariance matrix S
  MatrixXd z_diff = Zsig.colwise() - z_pred;
  MatrixXd R = MatrixXd(n_z, n_z);
  R << std_laspx_*std_laspx_, 0, 
       0, std_laspy_*std_laspy_;
  MatrixXd weighted_dist =  (z_diff).array().rowwise() * weights_.array().transpose();
  S << ( weighted_dist * z_diff.transpose()) + R;
  
  /** Update state given measurement data*/
  //create vector for incoming radar measurement
  VectorXd z = VectorXd(n_z);
  z << meas_package.raw_measurements_; //px, and py
  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  //calculate cross correlation matrix
  MatrixXd weighted_diff_x = (Xsig_pred_.colwise() - x_).array().rowwise() * weights_.array().transpose();
  //Add correct phi to remains in the range -PI/+PI
  Tc <<  weighted_diff_x * z_diff.transpose();
  //calculate Kalman gain K;
  MatrixXd K = Tc * S.inverse();
  //update state mean and covariance matrix
  x_ = x_ + K*(z - z_pred);
  P_ = P_ - K * S * K.transpose();
 
  //Calculate NIS
  NIS_laser_ = ( ( z - z_pred).transpose() ) * S.inverse() * ( z - z_pred );
  //cout << "Laser NIS " << NIS_laser_ << endl;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /** Convert sigma points from process model space to measurement model space */
  //set measurement dimension of radar (r, phi, r_dot)
  int n_z = 3;
  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, n_sigma_pts);
  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  //transform sigma points into measurement space
  for(int i=0; i< (2*n_aug_ + 1); i++){
      double px = Xsig_pred_(0,i);
      double py = Xsig_pred_(1,i);
      double v = Xsig_pred_(2,i);
      double psi = Xsig_pred_(3,i);
      Zsig(0, i) = sqrt(px*px + py*py);
      Zsig(1, i) = atan2(py, px);
      Zsig(2, i) = (px *cos(psi) * v + py * sin(psi) * v)/sqrt(px*px + py*py);
  }
  //calculate mean predicted measurement
  z_pred = Zsig * weights_;
  //calculate measurement covariance matrix S
  MatrixXd z_diff = Zsig.colwise() - z_pred;
  //cout<<"********** z diff size " << z_diff.size();
  MatrixXd R = MatrixXd(n_z, n_z);
  R << std_radr_*std_radr_, 0, 0, 
       0, std_radphi_*std_radphi_, 0,
       0,0, std_radrd_*std_radrd_;
  //Keep angle psi in range [-PI, PI]
  for(int i=0; i<n_sigma_pts; i++){
    while (z_diff(1, i)> M_PI) z_diff(1, i)-=2.*M_PI;
    while (z_diff(1, i)<-M_PI) z_diff(1, i)+=2.*M_PI;
  }
  MatrixXd weighted_dist =  (z_diff).array().rowwise() * weights_.array().transpose();
  S << ( weighted_dist * z_diff.transpose()) + R;
  
  /** Update state given measurement data*/
  //create example vector for incoming radar measurement
  VectorXd z = VectorXd(n_z);
  z << meas_package.raw_measurements_;
  // measurements components rho [m], phi [rad], rho_dot [m/s]
  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  //calculate cross correlation matrix
  MatrixXd weighted_diff_x = (Xsig_pred_.colwise() - x_).array().rowwise() * weights_.array().transpose();
  //Add correct phi to remains in the range -PI/+PI
  Tc <<  weighted_diff_x * z_diff.transpose();
  //calculate Kalman gain K;
  MatrixXd K = Tc * S.inverse();
  //update state mean and covariance matrix
  x_ = x_ + K*(z - z_pred);
  P_ = P_ - K * S *K.transpose();
  NIS_radar_ = ( ( z - z_pred).transpose() ) * S.inverse() * ( z - z_pred );
}
