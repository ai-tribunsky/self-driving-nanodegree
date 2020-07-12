#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include <vector>
#include <map>

//#include "Eigen-3.3/Eigen/Core"
//#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "helpers.h"

#include "PathPlanner/Config.h"
#include "PathPlanner/Planner.h"
#include "PathPlanner/Trajectory.h"


// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using std::map;

int main() {
    uWS::Hub h;

    // Load up map values for waypoints x,y,s and d normalized normal vectors
    PathPlanner::Map map = PathPlanner::Map::buildFromFile(
            PathPlanner::MAP_FILE_NAME,
            PathPlanner::MAP_WAYPOINTS_COUNT,
            PathPlanner::MAP_LANES_COUNT,
            PathPlanner::MAP_LANE_WIDTH,
            PathPlanner::MAX_VELOCITY
    );
    PathPlanner::Planner planner(map, PathPlanner::PLANNING_HORIZON);

    auto ws_message_handler = [&planner](uWS::WebSocket<uWS::SERVER> ws, char *message, size_t length, uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length > 2 && message[0] == '4' && message[1] == '2') {
            const auto s = hasData(message);
            if (!s.empty()) {
                // Automatic driving
                const auto j = json::parse(s);
                const string event = j[0].get<string>();
                if (event == "telemetry") {
                    // j[1] is the data JSON object
                    PathPlanner::Trajectory trajectory = planner.run(j[1]);

                    json msgJson;
                    msgJson["next_x"] = trajectory.x;
                    msgJson["next_y"] = trajectory.y;

                    std::string msg{"42[\"control\"," + msgJson.dump() + "]"};
                    ws.send(msg.c_str(), msg.length(), uWS::OpCode::TEXT);
                }
            } else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }  // end websocket if
    };

    auto ws_connection_handler = [&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    };

    auto ws_disconnect_handler = [&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    };

    h.onMessage(ws_message_handler);
    h.onConnection(ws_connection_handler);
    h.onDisconnection(ws_disconnect_handler);

    int port{4567};
    if (h.listen(port)) {
        std::cout << "Listening to port " << port << std::endl;
        h.run();
    }

    std::cerr << "Failed to listen to port " << port << std::endl;
    return -1;
}