#include "particle_filter.h"

#include <cmath>
#include <algorithm>
#include <iterator>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;

ParticleFilter::ParticleFilter(unsigned int num_particles) : _num_particles(num_particles), _is_initialized(false) {}

void ParticleFilter::init(double x, double y, double theta, const double std[]) {
    _particles.reserve(_num_particles);
    _weights.reserve(_num_particles);

    std::default_random_engine generator(std::random_device{}());
    std::normal_distribution<double> x_distribution(x, std[0]);
    std::normal_distribution<double> y_distribution(y, std[1]);
    std::normal_distribution<double> theta_distribution(theta, std[2]);

    const double weight = 1.0;
    for (unsigned int i = 0; i < _num_particles; i++) {
        Particle particle{
                i,
                x_distribution(generator),
                y_distribution(generator),
                theta_distribution(generator),
                weight
        };
        _particles.push_back(particle);
        _weights.push_back(weight);
    }

    _is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, const double std_pos[], double velocity, double yaw_rate) {
    std::random_device rd;
    std::default_random_engine generator(rd());
    std::normal_distribution<double> x_distribution(0.0, std_pos[0]);
    std::normal_distribution<double> y_distribution(0.0, std_pos[1]);
    std::normal_distribution<double> theta_distribution(0.0, std_pos[2]);

    double velocity_rate, x, y, theta;
    for (auto &particle: _particles) {
        if (fabs(yaw_rate) > 0.00001) {
            velocity_rate = (velocity / yaw_rate);
            x = particle.x + velocity_rate * (std::sin(particle.theta + yaw_rate * delta_t) - std::sin(particle.theta));
            y = particle.y + velocity_rate * (std::cos(particle.theta) - std::cos(particle.theta + yaw_rate * delta_t));
            theta = particle.theta + yaw_rate * delta_t;
        } else {
            x = particle.x + velocity * delta_t * std::cos(particle.theta);
            y = particle.y + velocity * delta_t * std::sin(particle.theta);
            theta = particle.theta;
        }

        particle.x = x + x_distribution(generator);
        particle.y = y + y_distribution(generator);
        particle.theta = theta + theta_distribution(generator);
    }
}

void ParticleFilter::updateWeights(double sensor_range, const double std_landmark[],
                                   const std::vector<LandmarkObs> &observations,
                                   const Map &map_landmarks) {
    const int observations_count = observations.size();
    const int landmarks_count = map_landmarks.landmark_list.size();

    double weight_sum{0.0};
    for (auto &particle: _particles) {
        vector<int> associations;
        vector<double> sense_x;
        vector<double> sense_y;
        associations.reserve(observations_count);
        sense_x.reserve(observations_count);
        sense_y.reserve(observations_count);

        vector<Map::single_landmark_s> sensor_range_landmarks;
        sensor_range_landmarks.reserve(landmarks_count);
        for (int i = 0; i < landmarks_count; i++) {
            const auto landmark = map_landmarks.landmark_list[i];
            const double particle_distance = dist(landmark.x_f, landmark.y_f, particle.x, particle.y);
            if (particle_distance <= sensor_range) {
                sensor_range_landmarks.push_back(landmark);
            }
        }

        double weight{1.0};
        for (const auto &observation: observations) {
            double obs_x = particle.x + (std::cos(particle.theta) * observation.x) -
                           (std::sin(particle.theta) * observation.y);
            double obs_y = particle.y + (std::sin(particle.theta) * observation.x) +
                           (std::cos(particle.theta) * observation.y);

            int landmark_idx{-1};
            double min_distance{sensor_range};
            for (const auto landmark: sensor_range_landmarks) {
                double distance = dist(landmark.x_f, landmark.y_f, obs_x, obs_y);
                if (distance < min_distance) {
                    min_distance = distance;
                    landmark_idx = landmark.id_i;
                }
            }

            associations.push_back(landmark_idx);
            const auto landmark = map_landmarks.landmark_list[landmark_idx - 1];
            weight *= multiv_prob(std_landmark[0], std_landmark[1], obs_x, obs_y, landmark.x_f, landmark.y_f);

            sense_x.push_back(obs_x);
            sense_y.push_back(obs_y);
        }

        particle.weight = weight;
        weight_sum += weight;

        SetAssociations(particle, associations, sense_x, sense_y);
    }

    // normalize particles weights
    for (unsigned int i = 0; i < _num_particles; i++) {
        if (weight_sum > 0.000001) {
            _particles[i].weight /= weight_sum;
            _weights[i] = _particles[i].weight;
        } else {
            _particles[i].weight = 1.0;
            _weights[i] = 1.0;
        }
    }
}

void ParticleFilter::resample() {
    std::random_device rd;
    std::default_random_engine generator(rd());
    std::discrete_distribution<> d(_weights.begin(), _weights.end());

    vector<Particle> sample;
    sample.reserve(_num_particles);

    for (unsigned int i = 0; i < _num_particles; i++) {
        int particle_idx = d(generator);
        sample.push_back(_particles[particle_idx]);
    }

    _particles.clear();
    _particles = sample;
}

void ParticleFilter::SetAssociations(Particle &particle,
                                     const vector<int> &associations,
                                     const vector<double> &sense_x,
                                     const vector<double> &sense_y) const {
    // particle: the particle to which assign each listed association,
    //   and association's (x,y) world coordinates mapping
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates
    particle.associations = associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(const Particle &best) const {
    vector<int> v = best.associations;
    std::stringstream ss;
    copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length() - 1);  // get rid of the trailing space
    return s;
}

string ParticleFilter::getSenseCoord(const Particle &best, const string &coord) const {
    vector<double> v;

    if (coord == "X") {
        v = best.sense_x;
    } else {
        v = best.sense_y;
    }

    std::stringstream ss;
    copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length() - 1);  // get rid of the trailing space
    return s;
}