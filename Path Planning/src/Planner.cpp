//
// Created by ai-tribunsky on 6/12/20.
//
#include "Planner.h"

const vector<BEHAVIOR_STATE> Planner::transitions[6]{
        // from STOP
        {STOP, KEEP_LANE},
        // from KEEP_LANE
        {STOP, KEEP_LANE, PREPARE_CHANGE_LANE_LEFT,  PREPARE_CHANGE_LANE_RIGHT},
        // from PREPARE_CHANGE_LANE_LEFT
        {STOP, KEEP_LANE, PREPARE_CHANGE_LANE_LEFT,  CHANGE_LANE_LEFT},
        // from PREPARE_CHANGE_LANE_RIGHT
        {STOP, KEEP_LANE, PREPARE_CHANGE_LANE_RIGHT, CHANGE_LANE_RIGHT},
        // from CHANGE_LANE_LEFT
        {STOP, KEEP_LANE, CHANGE_LANE_LEFT},
        // from CHANGE_LANE_RIGHT
        {STOP, KEEP_LANE, CHANGE_LANE_RIGHT}
};

Trajectory Planner::getTrajectory(
        const Trajectory &prev_trajectory,
        const EgoState &ego_state,
        const Map &map,
        const unordered_map<int, vector<vector<double>>> &cars
) {
    size_t prev_tj_points_count = prev_trajectory.x.size();
    double ref_x = ego_state.x;
    double ref_y = ego_state.y;
    if (prev_tj_points_count > 0) {
        ref_x = prev_trajectory.x[prev_tj_points_count - 1];
        ref_y = prev_trajectory.y[prev_tj_points_count - 1];
    }

    // get available states from current state
    vector<BEHAVIOR_STATE> states = transitions[current_state];
    vector<Trajectory> next_trajectories;
    next_trajectories.reserve(states.size());
    for (auto next_state: states) {
        // build trajectories for every state with costs
        //  - connected with previous path
        //  - obeys rules (max velocity)
        //  - obeys comfort rules (max acceleration, max jerk)
        //  - collision free
        //  - effective (maximize velocity)
        Trajectory possible_trajectory;
        switch (next_state) {
//            case STOP:
//                possible_trajectory = stop(prev_trajectory, ego_state, map, cars, ref_x, ref_y);
//                break;

            case KEEP_LANE:
                possible_trajectory = keep_lane(prev_trajectory, ego_state, map, cars, ref_x, ref_y);
                break;

//            case PREPARE_CHANGE_LANE_LEFT:
//                possible_trajectory = prepare_change_lane_left(prev_trajectory, ego_state, map, cars, ref_x, ref_y);
//                break;
//
//            case PREPARE_CHANGE_LANE_RIGHT:
//                possible_trajectory = prepare_change_lane_right(prev_trajectory, ego_state, map, cars, ref_x, ref_y);
//                break;
//
//            case CHANGE_LANE_LEFT:
//                possible_trajectory = change_lane_left(prev_trajectory, ego_state, map, cars, ref_x, ref_y);
//                break;
//
//            case CHANGE_LANE_RIGHT:
//                possible_trajectory = change_lane_right(prev_trajectory, ego_state, map, cars, ref_x, ref_y);
//                break;
        }

        next_trajectories.push_back(possible_trajectory);
    }

    // choose min cost trajectory
    sort(next_trajectories.begin(), next_trajectories.end());
    Trajectory min_cost_trajectory = next_trajectories[0];
    current_state = min_cost_trajectory.state;

    previous_cars_states = cars;

    return min_cost_trajectory;
}

Trajectory Planner::stop(const Trajectory &prev_trajectory,
                         const EgoState &ego_state,
                         const Map &map,
                         const unordered_map<int, vector<vector<double>>> &cars,
                         double ref_x,
                         double ref_y
) const {
    return prev_trajectory;
}

Trajectory Planner::keep_lane(const Trajectory &prev_trajectory,
                              const EgoState &ego_state,
                              const Map &map,
                              const unordered_map<int, vector<vector<double>>> &cars,
                              double ref_x,
                              double ref_y
) const {
    // anchors points for trajectory spline
    vector<double> tj_anchors_points_x{ref_x};
    vector<double> tj_anchors_points_y{ref_y};
    tj_anchors_points_x.reserve(4);
    tj_anchors_points_y.reserve(4);

    double target_velocity = max_velocity;
    if (cars.find(ego_state.lane) != cars.end()) {
        double min_distance = 1000.0;
        vector<vector<double>> cars_in_ego_lane = cars[ego_state.lane];
        for (const auto &car: cars_in_ego_lane) {
            // [ id, x, y, vx, vy, s, d]
            double dist = distance(ego_state.d, ego_state.s, car[6], car[5]);
            if (dist < 20.0 && dist < min_distance) {
                min_distance = dist;
                target_velocity = sqrt(car[3] * car[3] + car[4] + car[4]);
            }
        }
        target_velocity = std::min(target_velocity, max_velocity);
        std::cout << "Target Velocity: " << target_velocity << std::endl;
    }

    int target_lane = ego_state.lane;
    double target_d = target_lane * map.lane_width + 0.5 * map.lane_width;

    // create additional anchors points for spline
    double s_start = ego_state.s;
    double s_step{30.0};
    for (int i = 1; i <= 3; i++) {
        pair<double, double> anchor_point = getXY(s_start + i * s_step, target_d, map.waypoints_s, map.waypoints_x,
                                                  map.waypoints_y);
        tj_anchors_points_x.push_back(anchor_point.first);
        tj_anchors_points_y.push_back(anchor_point.second);
    }

    // build spline
    tk::spline spline;
    spline.set_points(tj_anchors_points_x, tj_anchors_points_y);

    Trajectory possible_trajectory;
    possible_trajectory.x = prev_trajectory.x;
    possible_trajectory.y = prev_trajectory.y;
    possible_trajectory.cost = 0.0;
    possible_trajectory.state = KEEP_LANE;

    size_t anchors_points_count = tj_anchors_points_x.size();
    double target_x = ref_x + 20;
    double target_y = spline(target_x);
    double target_distance = distance(ref_x, ref_y, target_x, target_y);
    int N = (int) ceil(target_distance / (target_velocity * time_step));
    double x_step = (target_x - ref_x) / N;
    size_t points_to_generate = N - prev_trajectory.x.size();
    for (size_t i = 1; i <= points_to_generate; i++) {
        double x = ref_x + x_step * i;
        double y = spline(x);

        possible_trajectory.x.push_back(x);
        possible_trajectory.y.push_back(y);
    }

    return possible_trajectory;
}

Trajectory Planner::prepare_change_lane_left(const Trajectory &prev_trajectory,
                                             const EgoState &ego_state,
                                             const Map &map,
                                             const unordered_map<int, vector<vector<double>>> &cars,
                                             double ref_x,
                                             double ref_y
) const {
    return prev_trajectory;
}

Trajectory Planner::prepare_change_lane_right(const Trajectory &prev_trajectory,
                                              const EgoState &ego_state,
                                              const Map &map,
                                              const unordered_map<int, vector<vector<double>>> &cars,
                                              double ref_x,
                                              double ref_y
) const {
    return prev_trajectory;
}

Trajectory Planner::change_lane_left(const Trajectory &prev_trajectory,
                                     const EgoState &ego_state,
                                     const Map &map,
                                     const unordered_map<int, vector<vector<double>>> &cars,
                                     double ref_x,
                                     double ref_y
) const {
    return prev_trajectory;
}

Trajectory Planner::change_lane_right(const Trajectory &prev_trajectory,
                                      const EgoState &ego_state,
                                      const Map &map,
                                      const unordered_map<int, vector<vector<double>>> &cars,
                                      double ref_x,
                                      double ref_y
) const {
    return prev_trajectory;
}
