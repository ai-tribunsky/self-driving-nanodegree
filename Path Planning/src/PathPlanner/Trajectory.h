//
// Created by ai-tribunsky on 7/10/20.
//

#ifndef PATH_PLANNING_TRAJECTORY_H
#define PATH_PLANNING_TRAJECTORY_H

namespace PathPlanner {

    struct State {
        double position;
        double velocity;
        double acceleration;
        double jerk;
        double timestamp;
    };

    class Trajectory {

    public:
        State start;
        State end;

    public:
        Trajectory() = default;

        Trajectory(const State &start, const State &end) : start(start), end(end) {}
    };

}
#endif //PATH_PLANNING_TRAJECTORY_H
