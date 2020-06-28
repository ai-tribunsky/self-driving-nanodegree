//
// Created by ai-tribunsky on 6/12/20.
//
#ifndef PATH_PLANNING_PLANNER_H
#define PATH_PLANNING_PLANNER_H

#include <unordered_map>
#include <vector>
#include <utility>
#include <algorithm>
#include <iostream>
#include <unordered_map>

#include "helpers.h"
#include "spline.h"


using std::vector;
using std::pair;
using std::unordered_map;

struct Map;
struct Trajectory;
struct Car;

enum BEHAVIOR_STATE {
    STOP,
    KEEP_LANE,
    PREPARE_CHANGE_LANE_LEFT,
    PREPARE_CHANGE_LANE_RIGHT,
    CHANGE_LANE_LEFT,
    CHANGE_LANE_RIGHT
};


class Planner {
public:
    Planner(double planner_horizon, double max_a, double max_j, double sim_time_step) :
            time_horizon(planner_horizon),
            max_acceleration(max_a),
            max_jerk(max_j),
            time_step(sim_time_step),
            current_state(STOP),
            ref_velocity(0.0) {
        max_velocity_delta = max_a * sim_time_step * 0.97;
        trajectory_points_count = (int) ceil(planner_horizon / sim_time_step);
    }

    Trajectory getTrajectory(
            const Trajectory &prev_trajectory,
            const Car &ego,
            const Map &map,
            const vector<Car> &cars
    );

private:
    Trajectory keep_lane(const Trajectory &prev_trajectory,
                         const Car &ego,
                         const Map &map,
                         const vector<double> &tj_start_points_x,
                         const vector<double> &tj_start_points_y,
                         double &ref_v,
                         double ref_x,
                         double ref_y,
                         double ref_yaw
    );

    double get_trajectory_cost(const Trajectory &trajectory, const Map &map) const;

    vector<vector<Trajectory>> get_cars_trajectories(const Map &map, const vector<Car> &cars);

private:
    // constraints
    double max_velocity_delta; // m/s
    double max_acceleration;   // m/s^2
    double max_jerk;           // m/s^3
    double time_horizon;
    int trajectory_points_count;

    // simulator options
    double time_step; // simulator rate in seconds

    // allowed transitions from every state
    static const vector<BEHAVIOR_STATE> transitions[6];
    BEHAVIOR_STATE current_state;

    // ego vehicle state
    double ref_velocity;

};

struct Car {
    double x;
    double y;
    double s;
    double d;
    double yaw;

    double vx;
    double vy;
    double v;

    double distance; // distance to ego vehicle
    int lane;

    bool operator<(const Car &str) const {
        if (lane == str.lane) {
            if (distance == str.distance) {
                return (v < str.v);
            }

            return distance < str.distance;
        }

        return (lane < str.lane);
    }

    void print() const {
        std::cout << "==== CAR ====";
        std::cout << "\n  x = " << x << "; y = " << y;
        std::cout << "\n  s = " << s << "; d = " << d;
        std::cout << "\n  vx = " << vx << "; vy = " << vy << "; v = " << v;
        std::cout << "\n  lane = " << lane;
        std::cout << "\n  distance = " << distance << '\n';
    }
};

struct Map {
    vector<double> waypoints_x;
    vector<double> waypoints_y;
    vector<double> waypoints_s;
    vector<double> waypoints_dx;
    vector<double> waypoints_dy;

    double track_length; // max s in Frenet coordinates
    double lane_width;   // meters
    int lanes_count;     // lanes count in forward direction

    double max_velocity;
    unordered_map<int, double> lanes_velocities;

    void updateLanesVelocities(vector<Car> &cars, const EgoState &ego) {
        // get minimal velocities of nearest cars
        auto begin = cars.begin();
        auto end = cars.end();
        sort(begin, end);

        for (int i = 0; i < lanes_count; ++i) {
            lanes_velocities[i] = max_velocity;
        }
        for (auto it = begin; it < end; ++it) {
            Car car = *it;
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
        std::cout << "\n  max_velocity = " << max_velocity;
        std::cout << "\n  Lanes Velocities:";
        for (const auto &item: lanes_velocities) {
            std::cout << "\n    " << item.first << " - " << item.second;
        }
        std::cout << '\n';
    }
};

struct Trajectory {
    vector<double> x;
    vector<double> y;
    vector<double> s;
    vector<double> d;

    BEHAVIOR_STATE state;
    double cost;
    float probability;

    void print() const {
        std::cout << "==== Trajectory ====";
        std::cout << "\n  state = " << state;
        std::cout << "\n  cost = " << cost;
        std::cout << "\n  probability = " << probability;
        auto it_x = x.cbegin();
        auto it_y = y.cbegin();
        for (; it_x != x.cend() && it_y != y.cend(); it_x++, it_y++) {
            std::cout << "\n  " << *it_x << ", " << *it_y;
        }
        std::cout << '\n';
    }

    bool operator<(const Trajectory &tr) const {
        return cost < tr.cost;
    }
};


#endif //PATH_PLANNING_PLANNER_H
