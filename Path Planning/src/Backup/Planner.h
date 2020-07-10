//
// Created by ai-tribunsky on 6/12/20.
//
#ifndef PATH_PLANNING_PLANNER_H
#define PATH_PLANNING_PLANNER_H

#include <vector>
#include <utility>
#include <iostream>
#include <map>
#include <unordered_map>

#include "../helpers.h"
#include "../spline.h"
#include "Map.h"
#include "Car.h"


enum BEHAVIOR_STATE {
    STOP,
    KEEP_LANE,
    PREPARE_CHANGE_LANE_LEFT,
    PREPARE_CHANGE_LANE_RIGHT,
    CHANGE_LANE_LEFT,
    CHANGE_LANE_RIGHT
};


class Trajectory {

public:
    Trajectory() : cost(1000.0), probability(1.0) {}

    Trajectory(const std::vector<double> &x, const std::vector<double> &y, double end_path_s, double end_path_d)
            : x(x), y(y), end_path_s(end_path_s), end_path_d(end_path_d), cost(1000.0), probability(1.0) {}

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
        if (probability == tr.probability) {
            return cost < tr.cost;
        }

        return probability > tr.probability;
    }

public:
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> s;
    std::vector<double> d;
    double end_path_s;
    double end_path_d;

    BEHAVIOR_STATE state;
    double cost;
    double probability;
};


class Planner {
public:
    Planner(
            double planner_time_horizon,
            double max_velocity,
            double max_acceleration,
            double max_jerk,
            double time_step
    );

    Trajectory run(const Map &map,
                   const Car &ego,
                   const std::map<int, Car> &cars,
                   const Trajectory &prev_trajectory
    );

private:
    // Updates cars likely trajectories using
    // data from sensors fusion data
    void track_cars(const Map &map, const std::map<CarId, Car> &cars);

    // Builds smooth trajectory which obeys velocity, acceleration, jerk constraints
    Trajectory build_trajectory(
            BEHAVIOR_STATE state,
            vector<double> start_x,
            vector<double> start_y,
            double target_d,
            double ref_velocity
    );

//    double get_trajectory_cost(const Trajectory &trajectory, const Map &map) const;

private:
    // constraints
    double max_velocity;       // m/s
    double max_velocity_delta; // m/s
    double max_acceleration;   // m/s^2
    double max_jerk;           // m/s^3
    double time_horizon;
    int trajectory_points_count;

    // simulator options
    double time_step; // simulator rate in seconds

    // allowed states_transitions from every state
    static const vector<BEHAVIOR_STATE> states_transitions[6];
    BEHAVIOR_STATE current_state;

    // ego vehicle state
    double ref_velocity;
    double ref_d;

    // world state
    double time;
    std::unordered_map<CarId, std::map<BEHAVIOR_STATE, Trajectory>> cars_trajectories;
    std::map<CarId, Car> cars_previous_states;
};


#endif //PATH_PLANNING_PLANNER_H
