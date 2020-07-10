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

        double s;    // Frenet's "s" coordinate
        double d;    // Frenet's "d" coordinate
        double yaw;  // yaw angle in radians

        double v;    // overall velocity in m/s
        double vx;   // x component of velocity in m/s
        double vy;   // y component of velocity in m/s

    public:
        Vehicle() = default;

        /**
         * Build vehicle from sensors fusion data
         * @param data - format [ id, x, y, vx, vy, s, d]
         */
        explicit Vehicle(const json &data) {
            id = data[0];
            vx = data[3];
            vy = data[4];
            v = sqrt(this->vx * this->vx + this->vy * this->vy);
            s = data[5];
            d = data[6];
            yaw = atan2(this->vy, this->vx);
        }

        /**
         * Update vehicle state from localization data
         * @param data
         */
        void updateState(const json &data) noexcept {
            this->s = data["s"];
            this->d = data["d"];
            this->yaw = deg2rad(data["yaw"]);
            this->v = mph2ms(data["speed"]);
        }
    };

}
#endif //PATH_PLANNING_VEHICLE_H
