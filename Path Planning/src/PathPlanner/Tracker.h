//
// Created by ai-tribunsky on 7/10/20.
//

#ifndef PATH_PLANNING_TRACKER_H
#define PATH_PLANNING_TRACKER_H

#include <unordered_map>
#include "Vehicle.h"
#include "Trajectory.h"
#include "Behaviors.h"


namespace PathPlanner {

    /**
     * Tracks states and predicts trajectories for vehicles around ego
     */
    class Tracker {

    public:

        void updateVehicleState(const Vehicle &vehicle) {
            vehicles[vehicle.id] = vehicle;
        }

        void update() {

        }

    private:
        // naive Bayes classificator features for "keep lane" behavior
        double kl_d_mean = 0.5;
        double kl_d_std = 0.25;
        double kl_d_rate_mean; // TODO: register observations of vehicle around and calculate mean ans std
        double kl_d_rate_std;  // TODO: register observations of vehicle around and calculate mean ans std
        double kl_s_rate_mean; // TODO: register observations of vehicle around and calculate mean ans std
        double kl_s_rate_std;  // TODO: register observations of vehicle around and calculate mean ans std

        // naive Bayes classificator features for "change lane left" behavior
        double cll_d_mean = 0.25;
        double cll_d_std = 0.25;
        double cll_d_rate_mean; // TODO: register observations of vehicle around and calculate mean ans std
        double cll_d_rate_std;  // TODO: register observations of vehicle around and calculate mean ans std
        double cll_s_rate_mean; // TODO: register observations of vehicle around and calculate mean ans std
        double cll_s_rate_std;  // TODO: register observations of vehicle around and calculate mean ans std

        // naive Bayes classificator features for "change lane right" behavior
        double clr_d_mean = 0.75;
        double clr_d_std = 0.25;
        double clr_d_rate_mean; // TODO: register observations of vehicle around and calculate mean ans std
        double clr_d_rate_std;  // TODO: register observations of vehicle around and calculate mean ans std
        double clr_s_rate_mean; // TODO: register observations of vehicle around and calculate mean ans std
        double clr_s_rate_std;  // TODO: register observations of vehicle around and calculate mean ans std

        std::unordered_map<VehicleId, Vehicle> vehicles;
        std::unordered_map<VehicleId, std::unordered_map<BEHAVIOR_STATE, Trajectory>> vehicles_trajectories;
    };

}
#endif //PATH_PLANNING_TRACKER_H
