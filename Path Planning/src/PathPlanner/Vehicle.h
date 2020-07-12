//
// Created by ai-tribunsky on 7/10/20.
//

#ifndef PATH_PLANNING_VEHICLE_H
#define PATH_PLANNING_VEHICLE_H

#include "../json.hpp"
#include "../helpers.h"


namespace PathPlanner {

    using json = nlohmann::json;
    typedef int VehicleId;

    /**
     * Class represents ego vehicle and vehicles around ego vehicle
     */
    class Vehicle {
    public:
        VehicleId id; // vehicle identificator

        // Cartesian
        double x;
        double y;
        double yaw; // yaw angle in radians
        double v;   // overall velocity in m/s
        double vx;  // x component of velocity in m/s
        double vy;  // y component of velocity in m/s

        // Frenet
        double s;
        double d;
        double vs{MAX_VELOCITY};
        double vd{0.0};
        double as{MAX_ACCELERATION};
        double ad{0.0};
        double js{MAX_JERK};
        double jd{0.0};

    public:
        Vehicle() = default;

        explicit Vehicle(VehicleId id): id(id) {}

        /**
         * Build vehicle from sensors fusion data
         * @param data - format [id, x, y, vx, vy, s, d]
         */
        explicit Vehicle(const json &data) {
            id = data[0];

            x = data[1];
            y = data[2];
            vx = data[3];
            vy = data[4];
            v = sqrt(vx * vx + vy * vy);
            yaw = atan2(vy, vx);

            s = data[5];
            d = data[6];
        }

        /**
         * Update vehicle state from localization data
         * @param data
         */
        void update(const json &data) {
            this->s = data["s"];
            this->d = data["d"];
            this->yaw = deg2rad(data["yaw"]);
            this->v = mph2ms(data["speed"]);
        }
    };

}
#endif //PATH_PLANNING_VEHICLE_H
