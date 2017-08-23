#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

#define EPS 0.0001

/***************************************************************************/
/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // set initialization state as false
  is_initialized_ = false;

  // initialize constant parameters
  n_x_ = 5;               // number of state variables
  n_aug_ = n_x_ + 2;      // augmentated state variable length
  n_sig_ = 2*n_aug_ + 1; // number of sigma points

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;
  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(n_x_);
  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);
  // initialize predicted sigma point matrix
  Xsig_pred_ = MatrixXd(n_x_, n_sig_);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1.25;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.75;

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

  /**
  TODO:
  Complete the initialization. See ukf.h for other member properties.
  Hint: one or more values initialized above might be wildly off...
  */

  // initialize lambda
  lambda_ = 3 - n_aug_;

  // initialize weights
  weights_ = VectorXd(n_sig_);
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  for (int i=1; i<n_sig_; i++) {
    weights_(i) = 0.5 / (n_aug_ + lambda_);
    }

  // initialize state vector
  x_ << 0.01, 0.01, 0, 0.001, 0;

  // initialize process covariance matrix
  P_.fill(0.0);
  P_(0,0) = 1;
  P_(1,1) = 1;
  P_(2,2) = 40;
  P_(3,3) = 1;
  P_(4,4) = 0.1;

  // initialize Lidar measurement noise covariance matrix
  R_laser_ = MatrixXd(2,2);
  R_laser_ << std_laspx_*std_laspx_, 0,
              0, std_laspy_*std_laspy_;

  // initialize radar measurement noise covariance matrix
  R_radar_ = MatrixXd(3,3);
  R_radar_.fill(0.0);
  R_radar_(0,0) = std_radr_*std_radr_;
  R_radar_(1,1) = std_radphi_*std_radphi_;
  R_radar_(2,2) = std_radrd_*std_radrd_;

  //cout << "DONE: initialized all process parameters" << endl;

}

UKF::~UKF() {}



/***************************************************************************/
/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:
  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  // CTRV Model with x_  = [px, py, vel, yaw-ang, yaw-ang_rate]
  if (!is_initialized_) {
    // first measurement
    //cout << "startig UKF: " << endl;

    // extract data for lidar & radar
    if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      //initialize lidar state
      float px = meas_package.raw_measurements_[0];
      float py = meas_package.raw_measurements_[1];
      // update lidar parameters
      x_ << px, py, 0, 0, 0;
      // special case with near zero values
      if ((fabs(x_(0)) < EPS) && (fabs(x_(1)) < EPS)) {
        x_(0) = EPS;
        x_(1) = EPS;
      }
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      //cout << "initializing RADAR" << endl;
      // convert radar from polar to cartesian and initialize
      // read data for radar
      float rho = meas_package.raw_measurements_[0]; //
      float phi = meas_package.raw_measurements_[1];
      float rho_dot = meas_package.raw_measurements_[2];
      // coordinate conversion
      float px = rho * cos(phi);
      float py = rho * sin(phi);
      float vx = rho_dot * cos(phi);
      float vy = rho_dot * sin(phi);
      float v = sqrt(vx*vx + vy*vy);
      // update radar parameters
      x_ << px, py, v, 0, 0;
    }

    // current time
    //cout << "ACTION: defining current time step" << endl;
    time_us_ = meas_package.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
    }

    //compute the time elapsed between the current and previous measurements
    //cout << "last time: " << time_us_ << endl;
    double dt = (meas_package.timestamp_ - time_us_) / 1000000.0;	//dt - expressed in seconds
    time_us_ = meas_package.timestamp_;
    //cout << "current time: " << time_us_ << endl;

    //  Predict
    Prediction(dt);

   // Update based on sensor-type
   //cout << "sensor-type: " << meas_package.sensor_type_ << endl;
  if ((meas_package.sensor_type_ == MeasurementPackage::LASER) && use_laser_) {
      // update based on lidar measurements
      //cout << "update lidar" << endl;
      UpdateLidar(meas_package);
   }
   else if ((meas_package.sensor_type_ == MeasurementPackage::RADAR) && use_radar_) {
     // update based on radar measurements
     //cout << "update radar" << endl;
     UpdateRadar(meas_package);
   }
   else {
      // neither radar nore lidar data available, through error
      cout << "ERROR: Neither Lidar nor Radar selected for update step" << endl;
   }
}



/***************************************************************************/
/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:
  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

  //////////////////////////////////////////////////////////////////////
  //create augmented mean state
  VectorXd x_aug = VectorXd(n_aug_);
  x_aug.fill(0.0);
  x_aug.head(n_x_) = x_;

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;

  //create square root matrix
  MatrixXd A = P_aug.llt().matrixL();

  //////////////////////////////////////////////////////////////////////
  // generate sigma points
  MatrixXd Xsig_aug = MatrixXd(n_aug_, n_sig_);
  //set first column of sigma point matrix
  Xsig_aug.col(0) = x_aug;
  //set remaining sigma points
  VectorXd x_aug_sigma_offset;
  for (int i = 0; i < n_aug_; i++) {
    x_aug_sigma_offset = sqrt(lambda_+n_aug_) * A.col(i);
    Xsig_aug.col(i+1)     = x_aug + x_aug_sigma_offset;
    Xsig_aug.col(i+1+n_aug_) = x_aug - x_aug_sigma_offset;
  }

  //////////////////////////////////////////////////////////////////////
  //cout << "starting sigma point prediction" << endl;
  // predict sigma points
  for (int i=0; i<n_sig_; i++) {
    double px = Xsig_aug(0,i);
    double py = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double psi = Xsig_aug(3,i);     // yaw
    double psiDot = Xsig_aug(4,i);  // yaw-rate
    double nu_a = Xsig_aug(5,i);    // noise in acceleration
    double nu_psi = Xsig_aug(6,i); // noise in yawdd

    // prediction update
    //cout << "performing prediction update" << endl;
    if (fabs(psiDot)>EPS) {
        px = px + v/psiDot*(sin(psi+psiDot*delta_t) - sin(psi));
        py = py + v/psiDot* (cos(psi) - cos(psi+psiDot*delta_t));
    }
    else {
        px = px + v*cos(psi)*delta_t;
        py = py + v*sin(psi)*delta_t;
    }
    double psi_new = psi + psiDot*delta_t;

    // noise update & writing prediction
    //cout << "adding process noise" << endl;
    Xsig_pred_(0,i) = px + (0.5)*(delta_t*delta_t) * cos(psi) * nu_a;
    Xsig_pred_(1,i) = py + (0.5)*(delta_t*delta_t) * sin(psi) * nu_a;
    Xsig_pred_(2,i) = v + delta_t*nu_a;
    Xsig_pred_(3,i) = psi_new + (0.5)*(delta_t*delta_t) * nu_psi;
    Xsig_pred_(4,i) = psiDot + delta_t*nu_psi;
  }

  //////////////////////////////////////////////////////////////////////
  // predict mean & covariance
  //cout << "predicting mean & covariance" << endl;
  x_ = Xsig_pred_ * weights_;
  P_.fill(0.0);
  for (int i=0; i<n_sig_; i++) {
      VectorXd xDelta = Xsig_pred_.col(i) - x_;
      // normalize angle
      while (xDelta(3) > M_PI) { xDelta(3) -= 2.0*M_PI; }
      while (xDelta(3) < -M_PI) { xDelta(3) += 2.0*M_PI; }
      //prediction of P_
      P_ = P_ + weights_(i)*xDelta*xDelta.transpose();
  }

  // prediction step complete

  //cout << "DONE: prediction complete" << endl;
}



/***************************************************************************/
/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:
  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.
  You'll also need to calculate the lidar NIS.
  */
  //set measurement dimension, lidar can measure x and y
  int n_z = 2;
  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z,n_z);
  //Zsig.fill(0.0);
  // transform z to measurement space
  Zsig = Xsig_pred_.block(0,0,n_z,n_sig_);
  /*for (int i=0; i<n_sig_; i++) {
    Zsig(0,i) = Xsig_pred__(0,i);
    Zsig(1,i) = Xsig_pred__(1,i);
  }*/

  // update Kalman Filter matrix, state vector and covariance matrix
  UpdateState_UKF(meas_package, Zsig, n_z);
}



/***************************************************************************/
/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:
  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.
  You'll also need to calculate the radar NIS.
  */
  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;
  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, n_sig_);
  Zsig.fill(0.0);

  //cout << "updating Zsig" << endl;
  // transform z to measurement space
  for (int i=0; i<n_sig_; i++) {
    double px = Xsig_pred_(0,i);
    double py = Xsig_pred_(1,i);
    double v = Xsig_pred_(2,i);
    double psi = Xsig_pred_(3,i);

    // check for near zero values
    if (fabs(px)<EPS and fabs(py)<EPS) {
      px = EPS;
      py = EPS;
    }

    // measurement vector
    Zsig(0,i) = sqrt(px*px + py*py);
    Zsig(1,i) = atan2(py, px);
    Zsig(2,i) = (px*v*cos(psi) + py*v*sin(psi)) / Zsig(0,i);
    }

    //cout << "Update completed: Zsig" << endl;
    //cout << "Zsig: " << Zsig << endl;

    // update Kalman Filter matrix, state vector and covariance matrix
    //cout << "updating radar state" << endl;
    UpdateState_UKF(meas_package, Zsig, n_z);
}


/***************************************************************************/
/**
   * Updates the state and the state covariance matrix based on specified
   measurement-package & measurement model (e.g., for radar, n_model = 3,
                                            while for lidar, n_model = 2)
   */
void UKF::UpdateState_UKF(MeasurementPackage meas_package, MatrixXd Zsig, int n_z) {


  ////////////////////////////////////////////////////////////////////////////
  // this might be common
  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  // predicted measurement mean
  z_pred.fill(0.0);
  for (int i=0; i<n_sig_; i++) {
      z_pred = z_pred + weights_(i)*Zsig.col(i);
  }

  // measurement covariance matrix
  MatrixXd S = MatrixXd(n_z,n_z);
  S.fill(0.0);
  for (int i=0; i<n_sig_; i++) {
      VectorXd zDelta = Zsig.col(i) - z_pred;
      // normalize angle
      while (zDelta(1) > M_PI) { zDelta(1) -= 2*M_PI; }
      while (zDelta(1) < -M_PI) { zDelta(1) += 2*M_PI; }
      // measurement covariance update
      S = S + weights_(i)*zDelta*zDelta.transpose();
  }

  // add measurement noise to covariance matrix
  //cout << "assign noise covariance matrix" << endl;
  MatrixXd R = MatrixXd(n_z,n_z);
  if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
    R = R_laser_;
  }
  else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    R = R_radar_;
  }

  /*
  cout << "add noise to measurement covariance: S = S+R" << endl;
  cout << "size: S: " << S.size() << endl;
  cout << "size: R: " << R.size() << endl;
  */
  S = S + R;

  //create matrix for cross correlation Tc
  //cout << "define cross-correlation matrix, Tc" << endl;
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i=0; i<n_sig_; i++) {
    // z residual
    VectorXd zDelta = Zsig.col(i) - z_pred;
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      // angle normalization
      while (zDelta(1) > M_PI) { zDelta(1) -= 2*M_PI; }
      while (zDelta(1) < -M_PI) { zDelta(1) += 2*M_PI; }
    }

    // x residual
    VectorXd xDelta = Xsig_pred_.col(i) - x_;
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      // angle normalization
      while (xDelta(3) > M_PI) { xDelta(3) -= 2*M_PI; }
      while (xDelta(3) < -M_PI) { xDelta(3) += 2*M_PI; }
    }

    // cross corelation column update
    Tc = Tc + weights_(i)*xDelta*zDelta.transpose();
  }

  // Kalman gain
  MatrixXd K = Tc * S.inverse();

  // z residual
  VectorXd z = meas_package.raw_measurements_; // measurements
  VectorXd zDelta = z-z_pred;
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    // angle normalization
    while (zDelta(1) > M_PI) { zDelta(1) -= 2*M_PI; }
    while (zDelta(1) < -M_PI) { zDelta(1) += 2*M_PI; }
  }

  //update state mean and covariance matrix
  x_ = x_ + K*zDelta;
  P_ = P_ - K*S*K.transpose();

  // NIS calculation for laser and radar
  if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
    NIS_laser_ = z.transpose() * S.inverse() * z;
  }
  else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    NIS_radar_ = z.transpose() * S.inverse() * z;
  }

  // DONE with update step
}
