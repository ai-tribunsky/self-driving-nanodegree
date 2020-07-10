//
// Created by ai-tribunsky on 6/12/20.
//
#include "Planner.h"
#include <algorithm>


const vector<BEHAVIOR_STATE> Planner::states_transitions[6]{
        // from STOP
        {STOP, KEEP_LANE},
        // from KEEP_LANE
        {STOP, KEEP_LANE, CHANGE_LANE_LEFT,          CHANGE_LANE_RIGHT},
        // from PREPARE_CHANGE_LANE_LEFT
        {STOP, KEEP_LANE, PREPARE_CHANGE_LANE_LEFT,  CHANGE_LANE_LEFT},
        // from PREPARE_CHANGE_LANE_RIGHT
        {STOP, KEEP_LANE, PREPARE_CHANGE_LANE_RIGHT, CHANGE_LANE_RIGHT},
        // from CHANGE_LANE_LEFT
        {STOP, KEEP_LANE, CHANGE_LANE_LEFT},
        // from CHANGE_LANE_RIGHT
        {STOP, KEEP_LANE, CHANGE_LANE_RIGHT}
};

Planner::Planner(
        double planner_time_horizon, double max_velocity, double max_acceleration, double max_jerk, double time_step
) :
        time_horizon(planner_time_horizon),
        max_acceleration(max_acceleration),
        max_jerk(max_jerk),
        time_step(time_step),
        current_state(STOP),
        ref_velocity(0.0),
        ref_d(0.0),
        time(0.0) {
    max_velocity_delta = max_acceleration * 0.97 * time_step;
    trajectory_points_count = (int) ceil(planner_time_horizon / time_step);
}

Trajectory Planner::run(
        const Map &map,
        const Car &ego,
        const std::map<CarId, Car> &cars,
        const Trajectory &prev_trajectory
) {
    // track cars around ego vehicle
    track_cars(map, cars);

    // run behavior planner every "time_horizon" intervals

    // build and follow trajectory based on behavior

    return {};
}

// Tracks other cars trajectories and assigns probabilities to trajectories
// using multimodal estimation: Pk = Pk-1 * Lk / normalizer
// https://classroom.udacity.com/nanodegrees/nd013/parts/01a340a5-39b5-4202-9f89-d96de8cf17be/modules/b74b8e43-47d1-47d6-a4cf-4d64ea3e0b80/lessons/9acfbc49-0e0f-44ed-8919-18602f69ff9a/concepts/043780c3-f412-4ed4-9212-7e457804f175
void Planner::track_cars(const Map &map, const std::map<CarId, Car> &cars) {
    if (!cars.empty()) {
        // TODO: track other cars data, label it and calculate typical d_rate, s_rate for maneuvers
        // features for naive base classifier
        // keep lane:
        //  d - mean = 0.5, std = 0.25
        //  d_rate - mean = ?, std = ? - near 0
        //  s_rate - mean = ?, std = ?
        //
        //  change lane left:
        //  d - mean = 0.25, std=0.25
        //  d_rate - mean = ?, std = ? - less than zero
        //  s_rate - mean = ?, std = ?
        //
        //  change lane right:
        //  d - mean = 0.75, std = 0.25
        //  d_rate - mean = ?, std = ? - more then zero
        //  s_rate - mean = ?, std = ?
        double d, d_rate, s_rate;

        // prior probabilities
        double keep_lane_prior{0.5};
        double change_lane_left_prior{0.25};
        double change_lane_right_prior{0.25};

        for (const auto &item: cars) {
            CarId car_id = item.first;
            Car car = item.second;

            d = fmod(car.d, map.lane_width) / map.lane_width;

            // create initial points for future trajectories
            vector<double> start_x{};
            vector<double> start_y{};
            if (cars_previous_states.find(car.id) != cars_previous_states.end()) {
                Car previous_car_state = cars_previous_states[car_id];

                // set classifier features
                d_rate = (car.d - previous_car_state.d) / time_step;
                s_rate = (car.s - previous_car_state.s) / time_step;

                // update previous probabilities
                if (cars_trajectories[car_id].find(KEEP_LANE) != cars_trajectories[car_id].end()) {
                    keep_lane_prior = cars_trajectories[car_id][KEEP_LANE].probability;
                }

                if (cars_trajectories[car_id].find(CHANGE_LANE_LEFT) != cars_trajectories[car_id].end()) {
                    change_lane_left_prior = cars_trajectories[car_id][CHANGE_LANE_LEFT].probability;
                }

                if (cars_trajectories[car_id].find(CHANGE_LANE_RIGHT) != cars_trajectories[car_id].end()) {
                    change_lane_right_prior = cars_trajectories[car_id][CHANGE_LANE_RIGHT].probability;
                }

                // set starting point for future trajectories
                start_x.push_back(previous_car_state.x);
                start_y.push_back(previous_car_state.y);
            }
            start_x.push_back(car.x);
            start_y.push_back(car.y);

            double lane_d = car.d;
            double left_lane_d = map.lane_width * (car.lane + 1.5);
            double right_lane_d = map.lane_width * (car.lane - 0.5);

            // keep lane
            Trajectory keep_lane = build_trajectory(KEEP_LANE, start_x, start_y, lane_d, car.v);
            keep_lane.probability = keep_lane_prior;
            keep_lane.probability *= getPdf(d, 0.5, 0.25);
            cars_trajectories[car_id][KEEP_LANE] = keep_lane;

            // change lane left
            Trajectory change_lane_left = build_trajectory(CHANGE_LANE_LEFT, start_x, start_y, left_lane_d, car.v);
            change_lane_left.probability = change_lane_left_prior;
            change_lane_left.probability *= getPdf(d, 0.25, 0.25);
            cars_trajectories[car_id][CHANGE_LANE_LEFT] = change_lane_left;

            // change lane right
            Trajectory change_lane_right = build_trajectory(CHANGE_LANE_RIGHT, start_x, start_y, right_lane_d, car.v);
            change_lane_right.probability = change_lane_right_prior;
            change_lane_right.probability *= getPdf(d, 0.75, 0.25);
            cars_trajectories[car_id][CHANGE_LANE_RIGHT] = change_lane_left;
        }
    }

    cars_previous_states = cars;
}

Trajectory Planner::build_trajectory(
        BEHAVIOR_STATE state,
        vector<double> start_x,
        vector<double> start_y,
        double target_d,
        double ref_velocity
) {
    //dx = (v + at)cos(yaw), dy = (v+at)sin(yaw)
    Trajectory trajectory;
    trajectory.state = state;



    return trajectory;
}

/*





Trajectory Planner::getTrajectory(
        const Trajectory &prev_trajectory,
        const Car &ego,
        const Map &map,
        const std::map<int, Car> &cars
) {
    if (time >= time_horizon) {
        run_behaviour_planner();
        time = 0.0;
    } else {
        time += time_step;
    }

    return run_trajectory_planner(prev_trajectory, ego, map, cars);


    // get available states from current state
    vector<BEHAVIOR_STATE> states = states_transitions[current_state];
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
                possible_trajectory = keep_lane(
                        prev_trajectory,
                        ego,
                        map,
                        tj_start_points_x,
                        tj_start_points_y,
                        ref_velocity,
                        ref_x,
                        ref_y,
                        ref_yaw
                );
                break;

//            case PREPARE_CHANGE_LANE_LEFT:
//                possible_trajectory = prepare_change_lane_left(prev_trajectory, ego_state, map, cars, ref_x, ref_y);
//                break;
//
//            case PREPARE_CHANGE_LANE_RIGHT:
//                possible_trajectory = prepare_change_lane_right(prev_trajectory, ego_state, map, cars, ref_x, ref_y);
//                break;
//
            case CHANGE_LANE_LEFT:
                if (ego.lane == 0) {
                    continue;
                }

                possible_trajectory = change_lane_left(
                        prev_trajectory,
                        ego,
                        map,
                        tj_start_points_x,
                        tj_start_points_y,
                        ref_velocity,
                        ref_x,
                        ref_y,
                        ref_yaw
                );
                break;

            case CHANGE_LANE_RIGHT:
                if (ego.lane == map.lanes_count - 1) {
                    continue;
                }

                possible_trajectory = change_lane_right(
                        prev_trajectory,
                        ego,
                        map,
                        tj_start_points_x,
                        tj_start_points_y,
                        ref_velocity,
                        ref_x,
                        ref_y,
                        ref_yaw
                );
                break;

            default:
                continue;
        }

        possible_trajectory.state = next_state;
        possible_trajectory.cost = get_trajectory_cost(possible_trajectory, map);
        next_trajectories.push_back(possible_trajectory);
    }

    // choose min cost trajectory
    sort(next_trajectories.begin(), next_trajectories.end());
    Trajectory min_cost_trajectory = next_trajectories[0];
    current_state = min_cost_trajectory.state;

    // update world state
    previous_cars = cars;

    return min_cost_trajectory;
}

void Planner::run_behaviour_planner() {

}

Trajectory Planner::run_trajectory_planner(
        const Trajectory &prev_trajectory,
        const Car &ego,
        const Map &map,
        const std::map<int, Car> &cars
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
            prev_ref_x = ego.x;
            prev_ref_y = ego.y;
            ref_yaw = ego.yaw;
        }
    } else {
        ref_x = ego.x;
        ref_y = ego.y;
        ref_yaw = ego.yaw;

        prev_ref_x = ref_x - cos(ref_yaw);
        prev_ref_y = ref_y - sin(ref_yaw);
    }
    tj_start_points_x.push_back(prev_ref_x);
    tj_start_points_y.push_back(prev_ref_y);
    tj_start_points_x.push_back(ref_x);
    tj_start_points_y.push_back(ref_y);

    switch (current_state) {
        case KEEP_LANE:

    }
}

Trajectory Planner::keep_lane(const Trajectory &prev_trajectory,
                              const Car &ego,
                              const Map &map,
                              const vector<double> &tj_start_points_x,
                              const vector<double> &tj_start_points_y,
                              double &ref_v,
                              double ref_x,
                              double ref_y,
                              double ref_yaw
) {
    std::cout << "\n\n====== KEEP LANE =====\n";
    map.print();
    ego.print();

    // anchors points for trajectory spline
    vector<double> tj_anchors_points_x{tj_start_points_x};
    vector<double> tj_anchors_points_y{tj_start_points_y};

    int target_lane = map.getLane(prev_trajectory.end_path_d);
    double target_d = target_lane * map.lane_width + 0.5 * map.lane_width;

    // create additional anchors points for spline
    double s_start = ego.s;
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

    size_t prev_tj_points_count = prev_trajectory.x.size();
    size_t points_to_generate = trajectory_points_count - prev_tj_points_count;

    double lane_velocity = map.lanes_velocities.at(ego.lane);

    double prev_x, prev_y;
    double x = ref_x;
    double y = ref_y;
    for (size_t i = 1; i <= points_to_generate; i++) {
        if (ref_v + max_velocity_delta <= lane_velocity) {
            ref_v += max_velocity_delta;
        } else if (ref_velocity > lane_velocity) {
            ref_v -= max_velocity_delta;
        } else {
            ref_v = lane_velocity;
        }

        prev_x = x;
        prev_y = y;

        x += ref_v * cos(ref_yaw) * time_step;
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
*/
