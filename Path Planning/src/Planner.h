//
// Created by ai-tribunsky on 6/12/20.
//
#include <unordered_map>
#include <vector>

#ifndef PATH_PLANNING_PLANNER_H
#define PATH_PLANNING_PLANNER_H

using std::vector;

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
    double cost;
};

struct Map {
    vector<double> waypoints_x;
    vector<double> waypoints_y;
    vector<double> waypoints_s;
    vector<double> waypoints_dx;
    vector<double> waypoints_dy;

    double track_length; // max s in Frenet coordinates
    double lane_width;  // meters
    int lanes_count;    // lanes count in forward direction
};

class Planner {
public:
    Planner(double max_v, double max_a, double max_j, double sim_time_step) :
            max_velocity(max_v),
            max_acceleration(max_a),
            max_jerk(max_j),
            time_step(sim_time_step),
            current_state(STOP) {}

    Trajectory getTrajectory(
            const Trajectory &prev_trajectory,
            const EgoState &state,
            const Map &map,
            const vector<vector<double>> &cars
    );

private:
    // constraints
    double max_velocity;     // m/s
    double max_acceleration; // m/s^2
    double max_jerk;         // m/s^3

    // simulator options
    double time_step; // simulator rate in seconds

    enum BEHAVIOR_STATE {
        STOP,
        KEEP_LANE,
        PREPARE_CHANGE_LANE_LEFT,
        PREPARE_CHANGE_LANE_RIGHT,
        CHANGE_LANE_LEFT,
        CHANGE_LANE_RIGHT
    };

    // allowed transitions from every state
    static const vector<BEHAVIOR_STATE> transitions[6];
    BEHAVIOR_STATE current_state;

    // world state
    vector<vector<double>> previous_cars_states;
};


#endif //PATH_PLANNING_PLANNER_H
