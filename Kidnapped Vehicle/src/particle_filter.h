#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_

#include <string>
#include <vector>
#include "helper_functions.h"

struct Particle {
    unsigned int id;
    double x;
    double y;
    double theta;
    double weight;
    std::vector<int> associations;
    std::vector<double> sense_x;
    std::vector<double> sense_y;
};


class ParticleFilter {
public:
    // Constructor
    // @param num_particles Number of particles
    explicit ParticleFilter(unsigned int num_particles);

    /**
     * Initializes particle filter by initializing particles to Gaussian
     *   distribution around first position and all the weights to 1.
     *
     * @param x Initial x position [m] (simulated estimate from GPS)
     * @param y Initial y position [m]
     * @param theta Initial orientation [rad]
     * @param std[] Array of dimension 3 [standard deviation of x [m],
     *   standard deviation of y [m], standard deviation of yaw [rad]]
     */
    void init(double x, double y, double theta, const double std[]);

    /**
     * Predicts the state for the next time step using the process model.
     *
     * @param delta_t Time between time step t and t+1 in measurements [s]
     * @param std_pos[] Array of dimension 3 [standard deviation of x [m],
     *   standard deviation of y [m], standard deviation of yaw [rad]]
     * @param velocity Velocity of car from t to t+1 [m/s]
     * @param yaw_rate Yaw rate of car from t to t+1 [rad/s]
     */
    void prediction(double delta_t, const double std_pos[], double velocity, double yaw_rate);

    /**
     * Updates the weights for each particle based on the likelihood
     *   of the observed measurements.
     *
     * @param sensor_range Range [m] of sensor
     * @param std_landmark[] Array of dimension 2
     *   [Landmark measurement uncertainty [x [m], y [m]]]
     * @param observations Vector of landmark observations
     * @param map Map class containing map landmarks
     */
    void updateWeights(double sensor_range, const double std_landmark[],
                       const std::vector<LandmarkObs> &observations,
                       const Map &map_landmarks);

    /**
     * Resamples from the updated set of particles to form
     *   the new set of particles.
     */
    void resample();

    /**
     * Set a particles list of associations, along with the associations'
     *   calculated world x,y coordinates
     * This can be a very useful debugging tool to make sure transformations
     *   are correct and associations correctly connected
     */
    void SetAssociations(Particle &particle, const std::vector<int> &associations,
                         const std::vector<double> &sense_x,
                         const std::vector<double> &sense_y) const;

    /**
     * Returns whether particle filter is initialized yet or not.
     */
    bool initialized() const {
        return _is_initialized;
    }

    /**
     * Used for obtaining debugging information related to particles.
     */
    std::string getAssociations(const Particle &best) const;

    std::string getSenseCoord(const Particle &best, const std::string &coord) const;

    // Set of current particles
    std::vector<Particle> _particles;

private:
    // Number of particles to draw
    unsigned int _num_particles;

    // Flag, if filter is initialized
    bool _is_initialized;

    // Vector of weights of all particles
    std::vector<double> _weights;
};

#endif  // PARTICLE_FILTER_H_