//
// Created by ai-tribunsky on 7/10/20.
//

#ifndef PATH_PLANNING_BEHAVIORPLANNER_H
#define PATH_PLANNING_BEHAVIORPLANNER_H

#include "Behaviors.h"
#include "Trajectory.h"

namespace PathPlanner {
    /**
     * Plans high level maneuvers like keep lane, change lane ans so on.
     * The next step action is selected according lower cost trajectory,
     * which is feasible, effective, obeys rules and comfort requirements.
     */
    class BehaviorPlanner {
    public:
        explicit BehaviorPlanner(double plan_horizon) : plan_horizon(plan_horizon) {}

        BEHAVIOR_STATE run() {}

    private:
        double plan_horizon;
    };
}
#endif //PATH_PLANNING_BEHAVIORPLANNER_H
