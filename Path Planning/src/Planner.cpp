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
        const vector<Car> &cars
) {
    // end of path reference point properties
    vector<double> tj_start_points_x;
    vector<double> tj_start_points_y;
    size_t prev_tj_points_count = prev_trajectory.x.size();
    double prev_ref_x, prev_ref_y;
    double ref_x, ref_y;
    double ref_yaw;
    if (prev_tj_points_count > 0) {
        ref_x = prev_trajectory.x[prev_tj_points_count - 1];
        ref_y = prev_trajectory.y[prev_tj_points_count - 1];

        if (prev_tj_points_count > 1) {
            prev_ref_x = prev_trajectory.x[prev_tj_points_count - 2];
            prev_ref_y = prev_trajectory.y[prev_tj_points_count - 2];
            ref_yaw = atan2(ref_y - prev_ref_y, ref_x - prev_ref_x);
        } else {
            prev_ref_x = ego_state.x;
            prev_ref_y = ego_state.y;
            ref_yaw = ego_state.yaw;
        }
    } else {
        ref_x = ego_state.x;
        ref_y = ego_state.y;
        ref_yaw = ego_state.yaw;

        prev_ref_x = ref_x - cos(ref_yaw);
        prev_ref_y = ref_y - sin(ref_yaw);
    }
    tj_start_points_x.push_back(prev_ref_x);
    tj_start_points_y.push_back(prev_ref_y);
    tj_start_points_x.push_back(ref_x);
    tj_start_points_y.push_back(ref_y);

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
//        Trajectory possible_trajectory;
        switch (next_state) {
//            case STOP:
//                possible_trajectory = stop(prev_trajectory, ego_state, map, cars, ref_x, ref_y);
//                break;

            case KEEP_LANE:
                Trajectory possible_trajectory = keep_lane(prev_trajectory, ego_state, map, cars, tj_start_points_x,
                                                           tj_start_points_y, ref_x, ref_y, ref_yaw);
                possible_trajectory.cost = get_trajectory_cost(possible_trajectory, map);

                possible_trajectory.print();
                next_trajectories.push_back(possible_trajectory);
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


    }

    // choose min cost trajectory
    sort(next_trajectories.begin(), next_trajectories.end());
    Trajectory min_cost_trajectory = next_trajectories[0];
    current_state = min_cost_trajectory.state;

    return min_cost_trajectory;
}

Trajectory Planner::keep_lane(const Trajectory &prev_trajectory,
                              const EgoState &ego_state,
                              const Map &map,
                              const vector<Car> &cars,
                              const vector<double> &tj_start_points_x,
                              const vector<double> &tj_start_points_y,
                              double ref_x,
                              double ref_y,
                              double ref_yaw
) {
    std::cout << "\n\n====== KEEP LANE =====\n";
    map.print();
    ego_state.print();

    // anchors points for trajectory spline
    vector<double> tj_anchors_points_x{tj_start_points_x};
    vector<double> tj_anchors_points_y{tj_start_points_y};

    int target_lane = ego_state.lane;
    double target_d = target_lane * map.lane_width + 0.5 * map.lane_width;

    // create additional anchors points for spline
    double s_start = ego_state.s;
    double s_step = 1.5 * map.max_velocity * trajectory_points_count * time_step;
    for (int i = 1; i <= 3; i++) {
        auto xy = getXY(s_start + i * s_step, target_d, map.waypoints_s, map.waypoints_x, map.waypoints_y);
        tj_anchors_points_x.push_back(xy.first);
        tj_anchors_points_y.push_back(xy.second);
    }

    // build spline
    tk::spline spline;
    spline.set_points(tj_anchors_points_x, tj_anchors_points_y);

    Trajectory possible_trajectory;
    possible_trajectory.x = prev_trajectory.x;
    possible_trajectory.y = prev_trajectory.y;
    possible_trajectory.cost = 0.0;
    possible_trajectory.state = KEEP_LANE;

    size_t prev_tj_points_count = prev_trajectory.x.size();
    size_t points_to_generate = trajectory_points_count - prev_tj_points_count;

    double lane_velocity = map.lanes_velocities.at(ego_state.lane);

    double prev_x, prev_y;
    double x = ref_x;
    double y = ref_y;
    for (size_t i = 1; i <= points_to_generate; i++) {
        if (ref_velocity + max_velocity_delta <= lane_velocity) {
            ref_velocity += max_velocity_delta;
        } else if (ref_velocity > lane_velocity) {
            ref_velocity -= max_velocity_delta;
        } else {
            ref_velocity = lane_velocity;
        }

        prev_x = x;
        prev_y = y;

        x += ref_velocity * cos(ref_yaw) * time_step;
        y = spline(x);
        ref_yaw = atan2(y - prev_y, x - prev_x);

        possible_trajectory.x.push_back(x);
        possible_trajectory.y.push_back(y);
    }

    return possible_trajectory;
}

double Planner::get_trajectory_cost(const Trajectory &trajectory, const Map &map) const {
    double cost{0.0};

    // evaluate trajectory efficiency
    double tj_max_velocity{0.0};
    double tj_max_acceleration{0.0};
    double tj_max_jerk{0.0};

    size_t waypoints_count = trajectory.x.size() - 1;
    double prev_velocity, velocity;
    double prev_acceleration, acceleration, jerk;
    double waypoints_distance;
    double t{0.0};
    for (size_t i = 0; i < waypoints_count; i += 2) {
        waypoints_distance = distance(trajectory.x[i], trajectory.y[i], trajectory.x[i + 1], trajectory.y[i + 1]);
        velocity = waypoints_distance / time_step;
        if (velocity > tj_max_velocity) {
            tj_max_velocity = velocity;
        }
        if (t > 0.0) {
            acceleration = std::abs(velocity - prev_velocity) / time_step;
            if (acceleration > tj_max_acceleration) {
                tj_max_acceleration = acceleration;
            }

            if (t > time_step) {
                jerk = std::abs(acceleration - prev_acceleration) / time_step;
                if (jerk > tj_max_jerk) {
                    tj_max_jerk = jerk;
                }
            }

            prev_acceleration = acceleration;
        }

        prev_velocity = velocity;
        t += time_step;
    }

    // penalize velocity overshoot more than undershoot
    double x = std::abs((tj_max_velocity - map.max_velocity) / map.max_velocity);
    cost += x * (x > 1 ? 2.0 : 1.0);
    // penalize acceleration overshoot only
    x = (tj_max_acceleration - max_acceleration) / max_acceleration;
    if (x > 1.0) {
        cost += x;
    }
    // penalize jerk overshoot only
    x = (tj_max_jerk - max_jerk) / max_jerk;
    if (x > 1.0) {
        cost += x;
    }

    return cost;
}