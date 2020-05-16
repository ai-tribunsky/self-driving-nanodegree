#include <iostream>

#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;


VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) const {
    VectorXd rmse(4);
    rmse << 0, 0, 0, 0;

    size_t count = estimations.size();
    if (count == 0 || count != ground_truth.size()) {
        return rmse;
    }

    for (int i = 0; i < count; ++i) {
        const VectorXd estimation = estimations[i];
        const VectorXd truth = ground_truth[i];

        rmse(0) += pow(truth(0) - estimation(0), 2);
        rmse(1) += pow(truth(1) - estimation(1), 2);
        rmse(2) += pow(truth(2) - estimation(2), 2);
        rmse(3) += pow(truth(3) - estimation(3), 2);
    }

    rmse(0) = sqrt(rmse(0) / count);
    rmse(1) = sqrt(rmse(1) / count);
    rmse(2) = sqrt(rmse(2) / count);
    rmse(3) = sqrt(rmse(3) / count);

    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd &x_state) const {
    MatrixXd Hj(3, 4);

    // recover state parameters
    const double px{x_state(0)};
    const double py{x_state(1)};
    const double vx{x_state(2)};
    const double vy{x_state(3)};

    // pre-compute a set of terms to avoid repeated calculation
    const double c1 = px * px + py * py;
    const double c2 = sqrt(c1);
    const double c3 = c1 * c2;

    // check division by zero
    if (c1 < 0.0001 || c2 < 0.0001 || c3 < 0.0001) {
        return Hj;
    }

    // compute the Jacobian matrix
    Hj << (px / c2), (py / c2), 0, 0,
            -(py / c1), (px / c1), 0, 0,
            py * (vx * py - vy * px) / c3, px * (px * vy - py * vx) / c3, px / c2, py / c2;

    return Hj;
}
