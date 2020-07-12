//
// Created by ai-tribunsky on 7/10/20.
//

#ifndef PATH_PLANNING_BEHAVIORPLANNER_H
#define PATH_PLANNING_BEHAVIORPLANNER_H

#include "Config.h"
#include "Vehicle.h"
#include "Tracker.h"
#include "Trajectory.h"


namespace PathPlanner {
    /**
     * Plans high level maneuvers like keep lane, change lane ans so on.
     * The next step action is selected according lower cost trajectory,
     * which is feasible, effective, obeys rules and comfort requirements.
     */
    class BehaviorPlanner {
    public:
        Map map;

    public:
        explicit BehaviorPlanner(Map map) : map(std::move(map)) {}

        BEHAVIOR_STATE run(const Vehicle &ego, const Tracker &tracker, double end_end_s, double path_end_d) {
            int ego_lane = map.getLane(ego.d);

            auto state_transitions = transitions[current_state];
            for (const auto &state: state_transitions) {
                switch (state) {
                    case STOP:
                        break;

                    case KEEP_LANE:
                        break;

                    case CHANGE_LANE_LEFT:
                        if (ego_lane == 0) { // leftmost lane
                            // trajectory not feasible
                            continue;
                        }


                        break;

                    case CHANGE_LANE_RIGHT:
                        if (ego_lane == map.lanes_count - 1) { // rightmost lane
                            // trajectory not feasible
                            continue;
                        }


                        break;
                }
            }
        }

    private:
        BEHAVIOR_STATE current_state{STOP};
    };
}
#endif //PATH_PLANNING_BEHAVIORPLANNER_H
