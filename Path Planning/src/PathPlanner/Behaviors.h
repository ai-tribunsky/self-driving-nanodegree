//
// Created by ai-tribunsky on 7/11/20.
//

#ifndef PATH_PLANNING_BEHAVIORS_H
#define PATH_PLANNING_BEHAVIORS_H

#include <unordered_map>


namespace PathPlanner {
    enum BEHAVIOR_STATE {
        STOP,
        KEEP_LANE,
        CHANGE_LANE_RIGHT,
        CHANGE_LANE_LEFT
    };

    std::unordered_map<BEHAVIOR_STATE, vector<BEHAVIOR_STATE>> transitions{
            {STOP,              {STOP, KEEP_LANE}},
            {KEEP_LANE,         {STOP, KEEP_LANE, CHANGE_LANE_LEFT, CHANGE_LANE_RIGHT}},
            {CHANGE_LANE_RIGHT, {STOP, KEEP_LANE}},
            {CHANGE_LANE_LEFT,  {STOP, KEEP_LANE}},
    };
}

#endif //PATH_PLANNING_BEHAVIORS_H
