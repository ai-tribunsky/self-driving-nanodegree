//
// Created by ai-tribunsky on 7/11/20.
//

#ifndef PATH_PLANNING_PLANNER_H
#define PATH_PLANNING_PLANNER_H

#include <utility>

#include "Trajectory.h"
#include "Map.h"
#include "Vehicle.h"
#include "Tracker.h"
#include "BehaviorPlanner.h"


namespace PathPlanner {

    using json = nlohmann::json;

    /**
     * Main class for path planner which enclose behavior planner, dynamic obstacles tracker and trajectories planner
     */
    class Planner {
    public:
        Planner(Map map, double planning_horizon) : map(std::move(map)), tracker(Tracker(map)),
                                                    planning_horizon(planning_horizon) {}

        Trajectory run(const json &data) {
            // update ego vehicle state
            ego.update(data);

            // update vehicles states around ego and their trajectories
            tracker.update(data["sensor_fusion"]);

            // previous path data given to the Planner
            auto previous_path_x = data["previous_path_x"];
            auto previous_path_y = data["previous_path_y"];
            double end_path_s = data["end_path_s"];
            double end_path_d = data["end_path_d"];

            if (previous_path_x.size() < 4) {
                behavior_planner.run(ego);
            }
        }

    private:
        Map map;
        Vehicle ego;
        Tracker tracker;

        // behavior planner
        double planning_horizon;
        BehaviorPlanner behavior_planner;

    };

}
#endif //PATH_PLANNING_PLANNER_H
