//
// Created by ai-tribunsky on 6/12/20.
//

#include "Planner.h"

const vector<Planner::BEHAVIOR_STATE> Planner::transitions[6]{
        // from STOP
        {Planner::STOP, Planner::KEEP_LANE},
        // from KEEP_LANE
        {Planner::STOP, Planner::KEEP_LANE, Planner::PREPARE_CHANGE_LANE_LEFT,  Planner::PREPARE_CHANGE_LANE_RIGHT},
        // from PREPARE_CHANGE_LANE_LEFT
        {Planner::STOP, Planner::KEEP_LANE, Planner::PREPARE_CHANGE_LANE_LEFT,  Planner::CHANGE_LANE_LEFT},
        // from PREPARE_CHANGE_LANE_RIGHT
        {Planner::STOP, Planner::KEEP_LANE, Planner::PREPARE_CHANGE_LANE_RIGHT, Planner::CHANGE_LANE_RIGHT},
        // from CHANGE_LANE_LEFT
        {Planner::STOP, Planner::KEEP_LANE, Planner::CHANGE_LANE_LEFT},
        // from CHANGE_LANE_RIGHT
        {Planner::STOP, Planner::KEEP_LANE, Planner::CHANGE_LANE_RIGHT}
};

Trajectory Planner::getTrajectory(
        const Trajectory &prev_trajectory,
        const EgoState &state,
        const Map &map,
        const vector<vector<double>> &cars
) {
    // get available states from current state

    // translate prev trajectory coords to Frenet

    // translate coordinate to ego's coordinate frame

    // build trajectories for every state with costs
    //  - connected with previous path
    //  - obeys rules (max velocity)
    //  - obeys comfort rules (max acceleration, max jerk)
    //  - collision free
    //  - effective (maximize velocity)

    // choose min cost trajectory

    // translate coordinates to world coordinate frame

    // translate trajectory coords from Frenet to x,y

    // return trajectory
}
