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
struct EgoState;
struct Trajectory;

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
    Planner(double planner_horizon, double max_v, double max_a, double max_j, double sim_time_step) :
            max_velocity(max_v),
            max_acceleration(max_a),
            max_jerk(max_j),
            time_step(sim_time_step),
            current_state(STOP),
            planner_time_horizon(planner_horizon) {
        max_trajectory_points_count = int(planner_horizon / time_step);
    }

    Trajectory getTrajectory(
            const Trajectory &prev_trajectory,
            const EgoState &ego_state,
            const Map &map,
            const vector<vector<double>> &cars
    );

private:
    Trajectory stop(const Trajectory &prev_trajectory,
                    const EgoState &ego_state,
                    const Map &map,
                    const vector<vector<double>> &cars,
                    double ref_x,
                    double ref_y
    ) const;

    Trajectory keep_lane(const Trajectory &prev_trajectory,
                         const EgoState &ego_state,
                         const Map &map,
                         const vector<vector<double>> &cars,
                         double ref_x,
                         double ref_y
    ) const;

    Trajectory prepare_change_lane_left(const Trajectory &prev_trajectory,
                                        const EgoState &ego_state,
                                        const Map &map,
                                        const vector<vector<double>> &cars,
                                        double ref_x,
                                        double ref_y
    ) const;

    Trajectory prepare_change_lane_right(const Trajectory &prev_trajectory,
                                         const EgoState &ego_state,
                                         const Map &map,
                                         const vector<vector<double>> &cars,
                                         double ref_x,
                                         double ref_y
    ) const;

    Trajectory change_lane_left(const Trajectory &prev_trajectory,
                                const EgoState &ego_state,
                                const Map &map,
                                const vector<vector<double>> &cars,
                                double ref_x,
                                double ref_y
    ) const;

    Trajectory change_lane_right(const Trajectory &prev_trajectory,
                                 const EgoState &ego_state,
                                 const Map &map,
                                 const vector<vector<double>> &cars,
                                 double ref_x,
                                 double ref_y
    ) const;

    vector<Trajectory> get_cars_trajectories(const vector<vector<double>> &cars, const Map &map, int steps_count) const;

    unordered_map<int, double> Planner::get_lanes_velocities(const EgoState &state, const Map& map, const vector<vector<double>> &cars) const;

private:
    // constraints
    double max_velocity;     // m/s
    double max_acceleration; // m/s^2
    double max_jerk;         // m/s^3

    double planner_time_horizon;

    // simulator options
    double time_step; // simulator rate in seconds

    // allowed transitions from every state
    static const vector<BEHAVIOR_STATE> transitions[6];
    BEHAVIOR_STATE current_state;

    // world state
    vector<vector<double>> previous_cars_states;
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
};

struct EgoState {
    double x;
    double y;
    double s;
    double d;
    double yaw;
    double speed;
    int lane;
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
        std::cout << "------------------\n";
        auto it_x = x.cbegin();
        auto it_y = y.cbegin();
        for (; it_x != x.cend() && it_y != y.cend(); it_x++, it_y++) {
            std::cout << "  " << *it_x << ", " << *it_y << '\n';
        }
    }

    bool operator<(const Trajectory &tr) const {
        return cost < tr.cost;
    }
};


#endif //PATH_PLANNING_PLANNER_H
