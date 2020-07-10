//
// Created by ai-tribunsky on 7/2/20.
//

#ifndef PATH_PLANNING_MAP_H
#define PATH_PLANNING_MAP_H

#include <vector>
#include <map>
#include "Car.h"

typedef int CarId;

class Map {

public:
    int getLane(double d) const {
        return (int) (d / lane_width);
    }

    void updateLanesVelocities(const std::map<CarId, Car> &cars, const Car &ego, double max_velocity) {
        lanes_velocities.clear();
        lanes_velocities.reserve(lanes_count);
        for (int i = 0; i < lanes_count; ++i) {
            lanes_velocities.push_back(max_velocity);
        }
        for (const auto &item: cars) {
            Car car = item.second;
            if (car.distance < 50.0) {
                if (car.s > ego.s) {
                    // other car in front of ego vehicle
                    lanes_velocities[car.lane] = std::min(lanes_velocities[car.lane], car.v);
                } else {
                    // other car behind ego vehicle
                    lanes_velocities[car.lane] = std::max(lanes_velocities[car.lane], car.v);
                }
            }
        }
    }

    void print() const {
        std::cout << "==== MAP ====";
        std::cout << "\n  track_length = " << track_length;
        std::cout << "\n  lane_width = " << lane_width;
        std::cout << "\n  lanes_count = " << lanes_count;
        std::cout << "\n  Lanes Velocities:";
        for (size_t i = 0; i < lanes_count; i++) {
            std::cout << "\n    " << i << " - " << lanes_velocities[i];
        }
        std::cout << '\n';
    }

public:
    std::vector<double> waypoints_x;
    std::vector<double> waypoints_y;
    std::vector<double> waypoints_s;
    std::vector<double> waypoints_dx;
    std::vector<double> waypoints_dy;

    double track_length; // max s in Frenet coordinates
    double lane_width;   // meters
    int lanes_count;     // lanes count in forward direction

    std::vector<double> lanes_velocities;
};

#endif //PATH_PLANNING_MAP_H
