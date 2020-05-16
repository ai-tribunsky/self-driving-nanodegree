#include <iostream>
#include <cmath>

#include "Eigen/Dense"
#include "FusionEKF.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::vector;


FusionEKF::FusionEKF() :
        is_initialized_(false),
        previous_timestamp_(0.0),
        tools({}),
        R_laser_(MatrixXd(2, 2)),
        R_radar_(MatrixXd(3, 3)),
        H_laser_(MatrixXd(2, 4)),
        noise_ax(11.0),
        noise_ay(11.0) {

    // LIDAR measurement covariance matrix
    R_laser_ << 0.0225, 0,
            0, 0.0225;
    // LIDAR measurement matrix
    H_laser_ << 1, 0, 0, 0,
            0, 1, 0, 0;
    // Radar measurement covariance matrix
    R_radar_ << 0.09, 0, 0,
            0, 0.09, 0,
            0, 0, 0.09;

    ekf_.I_ = MatrixXd::Identity(4, 4);

    ekf_.x_ = VectorXd(4);
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;

    ekf_.F_ = MatrixXd(4, 4);
    ekf_.F_ << 1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;
    ekf_.Q_ = MatrixXd(4, 4);
}

VectorXd FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
    if (!is_initialized_) {
        // first measurement
        cout << "EKF: \n";
        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
            const double ro = measurement_pack.raw_measurements_(0);
            const double theta = measurement_pack.raw_measurements_(1);
            ekf_.x_ << ro * cos(theta), ro * sin(theta), 0, 0;
        } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
            ekf_.x_ << measurement_pack.raw_measurements_(0), measurement_pack.raw_measurements_(1), 0, 0;
        }

        previous_timestamp_ = measurement_pack.timestamp_;

        // done initializing, no need to predict or update
        is_initialized_ = true;
        return ekf_.x_;
    }

    /**
     * Prediction
     */
    const double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
    previous_timestamp_ = measurement_pack.timestamp_;

    ekf_.F_(0, 2) = dt;
    ekf_.F_(1, 3) = dt;

    const double dt_2{dt * dt};
    const double dt_3{dt_2 * dt};
    const double dt_4{dt_3 * dt};
    ekf_.Q_ << dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0,
            0, dt_4 / 4 * noise_ay, 0, dt_3 / 2 * noise_ay,
            dt_3 / 2 * noise_ax, 0, dt_2 * noise_ax, 0,
            0, dt_3 / 2 * noise_ay, 0, dt_2 * noise_ay;
    ekf_.Predict();

    /**
     * Update
     */
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
        ekf_.R_ = R_radar_;
        ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    } else {
        ekf_.H_ = H_laser_;
        ekf_.R_ = R_laser_;
        ekf_.Update(measurement_pack.raw_measurements_);
    }

    // print the output
    cout << "x_ = " << ekf_.x_ << '\n';
    cout << "P_ = " << ekf_.P_ << '\n';
    return ekf_.x_;
}
