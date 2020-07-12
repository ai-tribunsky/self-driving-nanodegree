//
// Created by ai-tribunsky on 7/10/20.
//

#ifndef PATH_PLANNING_MAP_H
#define PATH_PLANNING_MAP_H

#include <vector>
#include <fstream>
#include <sstream>


namespace PathPlanner {
    /**
     * Class represents world map and contains world, road properties
     * and coordinate systems conversions methods
     */
    class Map {
    public:
        size_t waypoints_count;
        std::vector<double> waypoints_x;
        std::vector<double> waypoints_y;
        std::vector<double> waypoints_s;
        std::vector<double> waypoints_dx;
        std::vector<double> waypoints_dy;

        // lanes properties
        int lanes_count;
        double lane_width; // in meters

        // rules
        double max_velocity; // in m/s

    public:
        Map(int lanes_count, double lane_width, int waypoints_count, double max_velocity) :
                lanes_count(lanes_count), lane_width(lane_width), waypoints_count(waypoints_count),
                max_velocity(max_velocity) {
            waypoints_x.reserve(waypoints_count);
            waypoints_y.reserve(waypoints_count);
            waypoints_s.reserve(waypoints_count);
            waypoints_dx.reserve(waypoints_count);
            waypoints_dy.reserve(waypoints_count);
        }

        /**
         * Parses map file and builds map object
         *
         * @param file_name
         * @param waypoints_count
         * @param lanes_count
         * @param lane_width
         * @return Map
         */
        static Map buildFromFile(
                const char *file_name, int waypoints_count, int lanes_count, double lane_width, double max_velocity
        ) {
            Map map(lanes_count, lane_width, waypoints_count, max_velocity);
            std::ifstream map_stream(file_name, std::ifstream::in);
            std::string line;
            double x, y, s, d_x, d_y;
            while (getline(map_stream, line)) {
                std::istringstream iss(line);
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

            return map;
        }

        int getLane(double d) const {
            return (int) (d / lane_width);
        }
    };

}
#endif //PATH_PLANNING_MAP_H
