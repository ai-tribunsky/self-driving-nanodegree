#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "Planner.h"


// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    size_t waypoints_count{181};
    Map map;
    map.waypoints_x.reserve(waypoints_count);
    map.waypoints_y.reserve(waypoints_count);
    map.waypoints_s.reserve(waypoints_count);
    map.waypoints_dx.reserve(waypoints_count);
    map.waypoints_dy.reserve(waypoints_count);

    // The max s value before wrapping around the track back to 0
    map.track_length = 6945.554;
    map.lane_width = 4.0;
    map.lanes_count = 3;

    // Waypoint map to read from
    const string map_file{"../data/highway_map.csv"};
    std::ifstream in_map(map_file.c_str(), std::ifstream::in);

    string line;
    while (getline(in_map, line)) {
        std::istringstream iss(line);
        double x, y, s, d_x, d_y;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        map.waypoints_x.push_back(x);
        map.waypoints_y.push_back(y);
        map.waypoints_s.push_back(s);
        map.waypoints_dx.push_back(d_x);
        map.waypoints_dy.push_back(d_y);
    }

    Planner *planner = new Planner(22.352, 10.0, 30.0, 0.02);

    uWS::Hub h;
    h.onMessage([&map, planner](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length > 2 && data[0] == '4' && data[1] == '2') {
            const auto s = hasData(data);
            if (!s.empty()) {
                const auto j = json::parse(s);
                string event = j[0].get<string>();
                if (event == "telemetry") {
                    // j[1] is the data JSON object

                    // Main car's localization Data
                    double car_x = j[1]["x"];
                    double car_y = j[1]["y"];
                    double car_s = j[1]["s"];
                    double car_d = j[1]["d"];
                    double car_yaw = j[1]["yaw"];
                    double car_speed = j[1]["speed"];

                    // Previous path data given to the Planner
                    auto previous_path_x = j[1]["previous_path_x"];
                    auto previous_path_y = j[1]["previous_path_y"];
                    // Previous path's end s and d values
                    double end_path_s = j[1]["end_path_s"];
                    double end_path_d = j[1]["end_path_d"];

                    // Sensor Fusion Data, a list of all other cars on the same side of the road.
                    auto sensor_fusion = j[1]["sensor_fusion"];

                    EgoState ego_state{car_x, car_y, car_s, car_d, car_yaw, car_speed, getLane(car_d, map.lane_width)};
                    Trajectory prev_trajectory{previous_path_x, previous_path_y};
                    Trajectory trajectory = planner->getTrajectory(prev_trajectory, ego_state, map, sensor_fusion);

                    json msgJson;
                    msgJson["next_x"] = trajectory.x;
                    msgJson["next_y"] = trajectory.y;

                    auto msg = "42[\"control\"," + msgJson.dump() + "]";

                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }  // end "telemetry" if
            } else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }  // end websocket if
    }); // end h.onMessage

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port{4567};
    if (h.listen(port)) {
        std::cout << "Listening to port " << port << std::endl;
        h.run();
    }

    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
}