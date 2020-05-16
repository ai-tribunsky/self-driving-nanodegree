#ifndef FusionEKF_H_
#define FusionEKF_H_

#include <fstream>
#include <string>
#include <vector>
#include "Eigen/Dense"
#include "kalman_filter.h"
#include "measurement_package.h"
#include "tools.h"

using Eigen::VectorXd;

class FusionEKF {
public:
    FusionEKF();

    /**
     * Run the whole flow of the Kalman Filter from here.
     */
    VectorXd ProcessMeasurement(const MeasurementPackage &measurement_pack);

    /**
     * Kalman Filter update and prediction math lives in here.
     */
    KalmanFilter ekf_;

private:
    // check whether the tracking toolbox was initialized or not (first measurement)
    bool is_initialized_;

    // previous timestamp
    long long previous_timestamp_;

    // tool object used to compute Jacobian and RMSE
    const Tools tools;

    Eigen::MatrixXd R_laser_;
    Eigen::MatrixXd R_radar_;
    Eigen::MatrixXd H_laser_;

    const double noise_ax;
    const double noise_ay;
};

#endif // FusionEKF_H_
