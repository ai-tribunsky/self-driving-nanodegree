//
// Created by ai-tribunsky on 7/11/20.
//

#ifndef PATH_PLANNING_CONFIG_H
#define PATH_PLANNING_CONFIG_H

#include <unordered_map>


namespace PathPlanner {
    // settings
    const char *MAP_FILE_NAME{"../data/highway_map.csv"};
    const int MAP_WAYPOINTS_COUNT{181};
    const int MAP_LANES_COUNT{3};
    const double MAP_LANE_WIDTH{4.0};   // meters
    const double PLANNING_HORIZON{2.0}; // seconds
    const double TRACKING_HORIZON{2.0}; // seconds
    const double SIM_TIME_STEP{0.02};   // seconds

    // constraints
    const double MAX_VELOCITY{22.2};     // m/s
    const double MAX_ACCELERATION{10.0}; // m/s^2
    const double MAX_JERK{10.0};         // m/s^3

    enum BEHAVIOR_STATE {
        STOP,
        KEEP_LANE,
        CHANGE_LANE_RIGHT,
        CHANGE_LANE_LEFT
    };

    std::unordered_map<BEHAVIOR_STATE, vector<BEHAVIOR_STATE>> transitions{
            {STOP,              {KEEP_LANE}},
            {KEEP_LANE,         {STOP, CHANGE_LANE_LEFT, CHANGE_LANE_RIGHT}},
            {CHANGE_LANE_RIGHT, {STOP, KEEP_LANE}},
            {CHANGE_LANE_LEFT,  {STOP, KEEP_LANE}},
    };
}

#endif //PATH_PLANNING_CONFIG_H
