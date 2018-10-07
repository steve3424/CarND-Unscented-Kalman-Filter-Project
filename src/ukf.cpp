#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
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
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  // set init to true after first measurement
  is_initialized_ = false;

  // num state dimensions
  n_x_ = 5;

  // num state dimensions + noise vector [std_a_, std_yawdd_]
  n_aug_ = 7;

  // lambda value
  lambda_ = 3 - n_aug_;

  // set Xsig_pred matrix size
  Xsig_pred_ = MatrixXd(n_x_,2*n_aug_+1);

  // set weight vector size
  weights_ = VectorXd(2*n_aug_+1);
}

UKF::~UKF() {}

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
	// check if first measurement
	if (!is_initialized_) {
		cout << "UKF first measurement initializing..." << endl;
		// initialize state vector x_ as 1's
		x_.fill(1.0);
		// initialize covariance P_ as identity matrix
		P_.setIdentity(5,5);

		// check measurement type and use_ variable
		if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
			// set state vector px and py with laser measurements
			x_(0) = meas_package.raw_measurements_(0);
			x_(1) = meas_package.raw_measurements_(1);	
		}
		else if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
			// convert radar measurements from polar to cartesian
			double px = cos(meas_package.raw_measurements_(1))*meas_package.raw_measurements_(0);
			double py = sin(meas_package.raw_measurements_(1))*meas_package.raw_measurements_(0);
			// set state vector px and py with radar measurements
			x_(0) = px;
			x_(1) = py;	

		}

		// finish first measurement initialization
		is_initialized_ = true;
		time_us_ = meas_package.timestamp_;
		return;
	}
	
	// calculate time change
	double delta_t = meas_package.timestamp_ - time_us_;

	// call prediction
	Prediction(delta_t);
}

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

	////////////////////////
	// GENERATE SIGMA POINTS
	////////////////////////
	
	// create augmented state vector
	VectorXd x_aug = VectorXd(n_aug_);
	x_aug.fill(0);
	x_aug.head(n_x_) = x_;

	// create augmented covariance matrix
	MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
	P_aug.fill(0.0);
	P_aug.topLeftCorner(n_x_, n_x_);
	P_aug(n_x_,n_x_) = std_a_*std_a_;
	P_aug(n_x_+1, n_x_+1) = std_yawdd_*std_yawdd_;

	// calculate square root matrix
	MatrixXd A = P_aug.llt().matrixL();
	
	// create augmented sigma points
	MatrixXd Xsig_aug = MatrixXd(n_aug_, 2*n_aug_+1);
	Xsig_aug.col(0) = x_aug;
	for (int i=0; i < n_aug_; ++i) {
		Xsig_aug.col(i+1) = x_aug + sqrt(lambda_+n_aug_)*A.col(i);
		Xsig_aug.col(i+n_aug_+1) = x_aug - sqrt(lambda_+n_aug_)*A.col(i);
	}
	
	////////////////////////
	// PREDICTE SIGMA POINTS
	////////////////////////
	
	// loop through each sigma point
	for (int i=0; i < 2*n_aug_+1; ++i) {
		// define intermediate state variables for readability
		double px = Xsig_aug(0,i);
		double py = Xsig_aug(1,i);
		double v = Xsig_aug(2,i);
		double yaw = Xsig_aug(3,i);
		double yaw_dot = Xsig_aug(4,i);
		double noise_a = Xsig_aug(5,i);
		double noise_yaw = Xsig_aug(6,i);

		// predicted location variables
		double px_pred, py_pred, v_pred, yaw_pred, yaw_dot_pred;

		// set px and py predictions and avoid 0 division error
		if (fabs(yaw_dot) > 0.001) {
			px_pred = px + v/yaw_dot*(sin(yaw + yaw_dot*delta_t) - sin(yaw));
			py_pred = py + v/yaw_dot*(-cos(yaw + yaw_dot*delta_t) + cos(yaw));
		}
		else {
			px_pred = v*cos(yaw)*delta_t;
			py_pred = v*sin(yaw)*delta_t;
		}

		// set v, yaw, and yaw_dot predictions
		v_pred = v;
		yaw_pred = yaw + yaw_dot*delta_t;
		yaw_dot_pred = yaw_dot;

		// add noise
		px_pred += 0.5*(delta_t*delta_t)*cos(yaw)*noise_a;
		py_pred += 0.5*(delta_t*delta_t)*sin(yaw)*noise_a;
		v_pred += delta_t*noise_a;
		yaw_pred += 0.5*(delta_t*delta_t)*noise_yaw;
		yaw_dot_pred += delta_t*noise_yaw;

		// add predicted sigma points to Xsig_pred matrix
		Xsig_pred_(0,i) = px_pred;
		Xsig_pred_(1,i) = py_pred;
		Xsig_pred_(2,i) = v_pred;
		Xsig_pred_(3,i) = yaw_pred;
		Xsig_pred_(4,i) = yaw_dot_pred;
	}

	//////////////////////////////
	// PREDICT MEAN AND COVARIANCE
	//////////////////////////////

	// set weight values
	double weight_0 = lambda_ / (lambda_+n_aug_);	
	weights_(0) = weight_0;
	for (int i=1; i < 2*n_aug_+1; ++i) {
		double weight_i = 0.5 / (lambda_ + n_aug_);
		weights_(i) = weight_i;
	}

	// predict mean
	VectorXd x_pred = VectorXd(n_x_);
	for (int i=0; i < 2*n_aug_+1; ++i) {
		x_pred += weights_(i)*Xsig_pred_.col(i);
	}
	// predict covariance
	MatrixXd P_pred = MatrixXd(n_x_, n_x_);
	for (int i=0; i < 2*n_aug_+1; ++i) {
		// calculate state diff
		VectorXd x_diff = Xsig_pred_.col(i) - x_pred;
		// normalize angle
		while (x_diff(3) > M_PI) {x_diff(3) -= 2*M_PI;}
		while (x_diff(3) < M_PI) {x_diff(3) += 2*M_PI;}
		// calculate P 
		P_pred += weights_(i)*x_diff*x_diff.transpose();
	}

	// set x_ and P_ to predicted mean and covariance
	x_ = x_pred;
	P_ = P_pred;
}

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
}

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
}
