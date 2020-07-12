//
// Created by ai-tribunsky on 7/10/20.
//

#ifndef PATH_PLANNING_TRACKER_H
#define PATH_PLANNING_TRACKER_H

#include <unordered_map>
#include "Vehicle.h"
#include "Trajectory.h"
#include "Config.h"


namespace PathPlanner {

    typedef Trajectory LongitudinalTrajectory;
    typedef Trajectory LateralTrajectory;


    /**
     * Tracks states and predicts trajectories for vehicles around ego
     */
    class Tracker {

    public:
        explicit Tracker(Map map) : map(std::move(map)) {}

        /**
         * Updates vehicles states and their potential trajectories (tracker state)
         * according data from sensors fusion
         *
         * @param sensor_fusion
         */
        void update(const json &sensor_fusion) {
            // prior probabilities
            double keep_lane_prior{0.5};
            double change_lane_left_prior{0.25};
            double change_lane_right_prior{0.25};

            // sensor_data = json[id, x, y, vx, vy, s, d]
            for (const auto &sensor_data: sensor_fusion) {
                // filter out not important vehicles
                if (sensor_data[6] < 0 || sensor_data[3] == 0 || sensor_data[4] == 0) {
                    continue;
                }

                VehicleId id = sensor_data[0];
                Vehicle vehicle(sensor_data);

                // update prior probabilities and classificator features
                double normalized_d = fmod(vehicle.d, map.lane_width) / map.lane_width;
                double velocity_d, velocity_s;
                if (vehicles.find(id) != vehicles.end()) {
                    Vehicle vehicle_previous_state = vehicles[id];

                    // update vehicle state
                    vehicle.vs = (vehicle.s - vehicle_previous_state.s) / SIM_TIME_STEP;
                    vehicle.vd = (vehicle.d - vehicle_previous_state.d) / SIM_TIME_STEP;
                    vehicle.as = (vehicle.vs - vehicle_previous_state.vs) / SIM_TIME_STEP;
                    vehicle.ad = (vehicle.vd - vehicle_previous_state.vd) / SIM_TIME_STEP;
                    vehicle.js = (vehicle.as - vehicle_previous_state.as) / SIM_TIME_STEP;
                    vehicle.jd = (vehicle.ad - vehicle_previous_state.ad) / SIM_TIME_STEP;

                    // update previous probabilities
                    if (probabilities[id].find(KEEP_LANE) != probabilities[id].end()) {
                        keep_lane_prior = probabilities[id][KEEP_LANE];
                    }

                    if (probabilities[id].find(CHANGE_LANE_LEFT) != probabilities[id].end()) {
                        change_lane_left_prior = probabilities[id][CHANGE_LANE_LEFT];
                    }

                    if (probabilities[id].find(CHANGE_LANE_RIGHT) != probabilities[id].end()) {
                        change_lane_right_prior = probabilities[id][CHANGE_LANE_RIGHT];
                    }
                }

                // build longitudinal trajectory
                State longitudinal_start;
                longitudinal_start.position = vehicle.s;
                longitudinal_start.velocity = vehicle.vs;
                longitudinal_start.acceleration = vehicle.as;
                longitudinal_start.jerk = vehicle.js;

                State longitudinal_end;
                longitudinal_start.position = vehicle.s + vehicle.vs * TRACKING_HORIZON + 0.5 * vehicle.as * TRACKING_HORIZON * TRACKING_HORIZON;
                longitudinal_start.jerk = vehicle.js;
                longitudinal_start.acceleration = vehicle.as + vehicle.js * TRACKING_HORIZON;
                longitudinal_start.velocity = vehicle.vs + longitudinal_start.acceleration * TRACKING_HORIZON;

                LongitudinalTrajectory longitudinal_tj(longitudinal_start, longitudinal_end);

                // build lateral trajectory
                State lateral_start;
                lateral_start.position = vehicle.d;
                lateral_start.velocity = vehicle.vd;
                lateral_start.acceleration = vehicle.ad;
                lateral_start.jerk = vehicle.jd;

                State lateral_end;
                longitudinal_start.jerk = vehicle.jd;
                longitudinal_start.acceleration = vehicle.ad + vehicle.jd * TRACKING_HORIZON;
                longitudinal_start.velocity = vehicle.vd + longitudinal_start.acceleration * TRACKING_HORIZON;

                // lateral: keep lane
                double keep_lane_d = (map.getLane(vehicle.d) + 0.5) * map.lane_width;
                lateral_end.position = keep_lane_d;
                LateralTrajectory keep_lane_tj(lateral_start, lateral_end);
                trajectories[id][KEEP_LANE] = {longitudinal_tj, keep_lane_tj};
                probabilities[id][KEEP_LANE] = keep_lane_prior * getPdf(normalized_d, kl_d_mean, kl_d_std);

                // lateral: change lane left
                lateral_end.position = keep_lane_d - map.lane_width;
                LateralTrajectory change_lane_left_tj(lateral_start, lateral_end);
                trajectories[id][CHANGE_LANE_LEFT] = {longitudinal_tj, change_lane_left_tj};
                probabilities[id][CHANGE_LANE_LEFT] = change_lane_left_prior * getPdf(normalized_d, cll_d_mean, cll_d_std);

                // lateral: change lane right
                lateral_end.position = keep_lane_d - map.lane_width;
                LateralTrajectory change_lane_right_tj(lateral_start, lateral_end);
                trajectories[id][CHANGE_LANE_RIGHT] = {longitudinal_tj, change_lane_right_tj};
                probabilities[id][CHANGE_LANE_RIGHT] = change_lane_right_prior * getPdf(normalized_d, clr_d_mean, clr_d_std);
            }
        }

        /**
         * Founds vehicle ahead of ego vehicle in specified lane
         *
         * @param ego - ego vehicle
         * @param lane - lane idx
         * @return vehicle ahead, if vehicle not found - id of vehicle will be equal -1
         */
        Vehicle getVehicleAhead(const Vehicle &ego, int lane) {
            Vehicle nearest_vehicle{-1};
            double distance, min_distance{1000.0};
            for (const auto& item: vehicles) {
                Vehicle vehicle = item.second;
                if (vehicle.s > ego.s && lane == map.getLane(vehicle.d)) {
                    distance = vehicle.s - ego.s;
                    if (distance < min_distance) {
                        nearest_vehicle = vehicle;
                        distance = vehicle.s;
                    }
                }
            }

            return nearest_vehicle;
        }

        /**
         * Founds vehicle ahead of ego vehicle in specified lane
         *
         * @param ego - ego vehicle
         * @param lane - lane idx
         * @return vehicle ahead, if vehicle not found - id of vehicle will be equal -1
         */
        Vehicle getVehicleBehind(const Vehicle &ego, int lane) {
            Vehicle nearest_vehicle{-1};
            double distance, min_distance{1000.0};
            for (const auto& item: vehicles) {
                Vehicle vehicle = item.second;
                if (vehicle.s < ego.s && lane == map.getLane(vehicle.d)) {
                    distance = ego.s - vehicle.s;
                    if (distance < min_distance) {
                        nearest_vehicle = vehicle;
                        distance = vehicle.s;
                    }
                }
            }

            return nearest_vehicle;
        }

        /**
         * Gets vehicle's most likely trajectories pair: longitudinal and lateral
         *
         * @param id
         * @return pair of trajectories
         */
        std::pair<LongitudinalTrajectory, LateralTrajectory> getVehicleTrajectories(VehicleId id) {
            if (probabilities.find(id) == probabilities.end()) {
                return {{}, {}};
            }

            std::unordered_map<BEHAVIOR_STATE, double> tj_probabilities = probabilities[id];
            BEHAVIOR_STATE max_state{KEEP_LANE};
            double max_probability{0.0};
            for (const auto &pair: tj_probabilities) {
                if (pair.second > max_probability) {
                    max_state = pair.first;
                    max_probability = pair.second;
                }
            }

            return trajectories[id][max_state];
        }

    private:
        Map map;

        // naive Bayes classificator features for "keep lane" behavior
        double kl_d_mean = 0.5;
        double kl_d_std = 0.25;
        double kl_velocity_d_mean; // TODO: register observations of vehicles around and calculate mean ans std
        double kl_velocity_d_std;  // TODO: register observations of vehicles around and calculate mean ans std
        double kl_velocity_s_mean; // TODO: register observations of vehicles around and calculate mean ans std
        double kl_velocity_s_std;  // TODO: register observations of vehicles around and calculate mean ans std

        // naive Bayes classificator features for "change lane left" behavior
        double cll_d_mean = 0.25;
        double cll_d_std = 0.25;
        double cll_velocity_d_mean; // TODO: register observations of vehicles around and calculate mean ans std
        double cll_velocity_d_std;  // TODO: register observations of vehicles around and calculate mean ans std
        double cll_velocity_s_mean; // TODO: register observations of vehicles around and calculate mean ans std
        double cll_velocity_s_std;  // TODO: register observations of vehicles around and calculate mean ans std

        // naive Bayes classificator features for "change lane right" behavior
        double clr_d_mean = 0.75;
        double clr_d_std = 0.25;
        double clr_velocity_d_mean; // TODO: register observations of vehicle around and calculate mean ans std
        double clr_velocity_d_std;  // TODO: register observations of vehicle around and calculate mean ans std
        double clr_velocity_s_mean; // TODO: register observations of vehicle around and calculate mean ans std
        double clr_velocity_s_std;  // TODO: register observations of vehicle around and calculate mean ans std

        std::unordered_map<VehicleId, Vehicle> vehicles;
        std::unordered_map<VehicleId, std::unordered_map<BEHAVIOR_STATE, std::pair<LongitudinalTrajectory, LateralTrajectory>>> trajectories;
        std::unordered_map<VehicleId, std::unordered_map<BEHAVIOR_STATE, double>> probabilities;
    };

}
#endif //PATH_PLANNING_TRACKER_H
