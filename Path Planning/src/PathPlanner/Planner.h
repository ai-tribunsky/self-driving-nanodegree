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
        Planner(Map map, double plan_horizon) : map(std::move(map)), behavior_planner(BehaviorPlanner(plan_horizon)) {

        }

        Trajectory run(const json &data) {
            // update ego vehicle state
            ego.updateState(data);

            // update vehicles states around ego and their trajectories
            auto sensor_fusion = data["sensor_fusion"];
            for (const auto &sensor_data: sensor_fusion) {
                // filter out not important cars
                // sensor_data = json[id, x, y, vx, vy, s, d]
                if (sensor_data[6] < 0 || sensor_data[3] == 0 || sensor_data[4] == 0) {
                    continue;
                }

                Vehicle vehicle(sensor_data);
                tracker.updateVehicleState(vehicle);
            }
            tracker.update();






            // Previous path data given to the Planner
            auto previous_path_x = data["previous_path_x"];
            auto previous_path_y = data["previous_path_y"];
            // Previous path's end s and d values
            double end_path_s = data["end_path_s"];
            double end_path_d = data["end_path_d"];


        }

    private:
        Map map;
        Vehicle ego;
        Tracker tracker;
        BehaviorPlanner behavior_planner;
    };

}
#endif //PATH_PLANNING_PLANNER_H
