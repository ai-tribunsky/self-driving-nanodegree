//
// Created by ai-tribunsky on 7/2/20.
//

#ifndef PATH_PLANNING_CAR_H
#define PATH_PLANNING_CAR_H

#include <iostream>

class Car {
public:
    bool operator<(const Car &str) const {
        if (lane == str.lane) {
            if (distance == str.distance) {
                return (v < str.v);
            }

            return distance < str.distance;
        }

        return (lane < str.lane);
    }

    void print() const {
        std::cout << "==== CAR ====";
        std::cout << "\n  x = " << x << "; y = " << y;
        std::cout << "\n  s = " << s << "; d = " << d;
        std::cout << "\n  vx = " << vx << "; vy = " << vy << "; v = " << v;
        std::cout << "\n  lane = " << lane;
        std::cout << "\n  distance = " << distance << '\n';
    }

public:
    int id;
    double x;
    double y;
    double s;
    double d;
    double yaw;

    double vx;
    double vy;
    double v;

    double distance; // distance to ego vehicle
    int lane;
};

#endif //PATH_PLANNING_CAR_H
