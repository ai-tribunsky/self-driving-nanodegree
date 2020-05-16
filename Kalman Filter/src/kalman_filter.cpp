#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() : tools({}) {
}

void KalmanFilter::Predict() {
    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
    const VectorXd z_pred = H_ * x_;
    const VectorXd y = z - z_pred;
    const MatrixXd Ht = H_.transpose();
    const MatrixXd S = H_ * P_ * Ht + R_;
    const MatrixXd Si = S.inverse();
    const MatrixXd PHt = P_ * Ht;
    const MatrixXd K = PHt * Si;

    // new estimate
    x_ = x_ + (K * y);
    P_ = (I_ - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
    double ro = sqrt(x_[0] * x_[0] + x_[1] * x_[1]);
    double theta = atan2(x_[1], x_[0]);
    double ro_dot = (x_[0] * x_[2] + x_[1] * x_[3]) / ro;

    VectorXd z_pred(3);
    z_pred << ro, theta, ro_dot;
    VectorXd y = z - z_pred;

    const MatrixXd Ht = H_.transpose();
    const MatrixXd S = H_ * P_ * Ht + R_;
    const MatrixXd Si = S.inverse();
    const MatrixXd PHt = P_ * Ht;
    const MatrixXd K = PHt * Si;

    // new estimate
    x_ = x_ + (K * y);
    P_ = (I_ - K * H_) * P_;
}
