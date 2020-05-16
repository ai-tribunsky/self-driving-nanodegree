#include <uWS/uWS.h>
#include <iostream>
#include <fstream>

#include "json.hpp"
#include "FusionEKF.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::string;
using std::vector;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(const string &s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of('[');
    auto b2 = s.find_first_of(']');
    if (found_null != string::npos) {
        return "";
    }
    if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 1);
    }
    return "";
}

MeasurementPackage readMeasurement(std::istringstream &iss) {
    MeasurementPackage meas_package;
    long long timestamp;

    // reads first element from the current line
    string sensor_type;
    iss >> sensor_type;

    if (sensor_type == "L") {
        meas_package.sensor_type_ = MeasurementPackage::LASER;
        meas_package.raw_measurements_ = VectorXd(2);
        double px;
        double py;
        iss >> px;
        iss >> py;
        meas_package.raw_measurements_ << px, py;
        iss >> timestamp;
        meas_package.timestamp_ = timestamp;
    } else if (sensor_type == "R") {
        meas_package.sensor_type_ = MeasurementPackage::RADAR;
        meas_package.raw_measurements_ = VectorXd(3);
        double ro;
        double theta;
        double ro_dot;
        iss >> ro;
        iss >> theta;
        iss >> ro_dot;
        meas_package.raw_measurements_ << ro, theta, ro_dot;
        iss >> timestamp;
        meas_package.timestamp_ = timestamp;
    }

    return meas_package;
}

VectorXd readGroundTruth(std::istringstream &iss) {
    double x_gt;
    double y_gt;
    double vx_gt;
    double vy_gt;
    iss >> x_gt;
    iss >> y_gt;
    iss >> vx_gt;
    iss >> vy_gt;

    VectorXd gt_values(4);
    gt_values(0) = x_gt;
    gt_values(1) = y_gt;
    gt_values(2) = vx_gt;
    gt_values(3) = vy_gt;
    return gt_values;
}


int main() {
    // Create a Kalman Filter instance
    FusionEKF fusionEKF;

    // used to compute the RMSE later
    const Tools tools;
    vector<VectorXd> estimations;
    vector<VectorXd> ground_truth;

//    std::ifstream file("../data/obj_pose-laser-radar-synthetic-input.txt");
//    if (file) {
//        string s;
//        while (std::getline(file, s)) {
//            std::istringstream iss(s);
//            const MeasurementPackage meas_package = readMeasurement(iss);
//            const VectorXd gt_values = readGroundTruth(iss);
//            ground_truth.push_back(gt_values);
//
//            // Call ProcessMeasurement(meas_package) for Kalman filter
//            VectorXd estimate = fusionEKF.ProcessMeasurement(meas_package);
//            estimations.push_back(estimate);
//
//            // RMSE should be less than or equal to the values [.11, .11, 0.52, 0.52]
//            VectorXd rmse = tools.CalculateRMSE(estimations, ground_truth);
//            std::cout << s << '\n';
//            std::cout << "px = " << estimate(0) << '\n';
//            std::cout << "py = " << estimate(1) << '\n';
//            std::cout << "px' = " << gt_values(0) << '\n';
//            std::cout << "py' = " << gt_values(1) << '\n';
//            std::cout << "RMSE = " << rmse(0) << ", " << rmse(1) << ", "
//                      << rmse(2) << ", " << rmse(3) << '\n';
//            std::cout << "---------------\n";
//        }
//        file.close();
//        return 0;
//    }
//
//    return -1;


    uWS::Hub h;
    h.onMessage([&fusionEKF, &tools, &estimations, &ground_truth]
                        (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                         uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length > 2 && data[0] == '4' && data[1] == '2') {
            const auto s = hasData(string(data));

            if (!s.empty()) {
                const auto j = json::parse(s);
                std::cout << "Input:" << j << '\n';

                string event = j[0].get<string>();
                if (event == "telemetry") {
                    // j[1] is the data JSON object
                    string sensor_measurement = j[1]["sensor_measurement"];
                    std::istringstream iss(sensor_measurement);

                    const MeasurementPackage meas_package = readMeasurement(iss);
                    const VectorXd gt_values = readGroundTruth(iss);
                    ground_truth.push_back(gt_values);

                    // Call ProcessMeasurement(meas_package) for Kalman filter
                    VectorXd estimate = fusionEKF.ProcessMeasurement(meas_package);
                    estimations.push_back(estimate);

                    // RMSE should be less than or equal to the values [.11, .11, 0.52, 0.52]
                    VectorXd rmse = tools.CalculateRMSE(estimations, ground_truth);

                    json msgJson;
                    msgJson["estimate_x"] = estimate(0);
                    msgJson["estimate_y"] = estimate(1);
                    msgJson["rmse_x"] = rmse(0);
                    msgJson["rmse_y"] = rmse(1);
                    msgJson["rmse_vx"] = rmse(2);
                    msgJson["rmse_vy"] = rmse(3);
                    auto msg = "42[\"estimate_marker\"," + msgJson.dump() + "]";
                    std::cout << msg << std::endl;
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

                }  // end "telemetry" if

            } else {
                string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }  // end websocket message if

    }); // end h.onMessage

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;
    if (h.listen(port)) {
        std::cout << "Listening to port " << port << std::endl;
    } else {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }

    h.run();
}