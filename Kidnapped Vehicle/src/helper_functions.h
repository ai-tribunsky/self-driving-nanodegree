/**
 * helper_functions.h
 * Some helper functions for the 2D particle filter.
 */

#ifndef HELPER_FUNCTIONS_H_
#define HELPER_FUNCTIONS_H_

#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "map.h"

// for portability of M_PI (Vis Studio, MinGW, etc.)
#ifndef M_PI
const double M_PI = 3.14159265358979323846;
#endif

/**
 * Struct representing one landmark observation measurement.
 */
struct LandmarkObs {
    double x;   // Local (vehicle coords) x position of landmark observation [m]
    double y;   // Local (vehicle coords) y position of landmark observation [m]
};

/**
 * Computes the Euclidean distance between two 2D points.
 * @param (x1,y1) x and y coordinates of first point
 * @param (x2,y2) x and y coordinates of second point
 * @output Euclidean distance between two 2D points
 */
inline double dist(const double x1, const double y1, const double x2, const double y2) {
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

/**
 * Computes the error between ground truth and particle filter data.
 * @param (gt_x, gt_y, gt_theta) x, y and theta of ground truth
 * @param (pf_x, pf_y, pf_theta) x, y and theta of particle filter
 * @output Error between ground truth and particle filter data.
 */
inline double *getError(double gt_x, double gt_y, double gt_theta, double pf_x, double pf_y, double pf_theta) {
    static double error[3];
    error[0] = fabs(pf_x - gt_x);
    error[1] = fabs(pf_y - gt_y);
    error[2] = fabs(pf_theta - gt_theta);
    error[2] = fmod(error[2], 2.0 * M_PI);
    if (error[2] > M_PI) {
        error[2] = 2.0 * M_PI - error[2];
    }
    return error;
}

/**
 * Reads map data from a file.
 * @param filename Name of file containing map data.
 * @output True if opening and reading file was successful
 */
inline bool read_map_data(const std::string &filename, Map &map) {
    // Get file of map
    std::ifstream in_file_map(filename.c_str(), std::ifstream::in);
    // Return if we can't open the file
    if (!in_file_map) {
        return false;
    }

    // Declare single line of map file
    std::string line_map;

    // Run over each single line
    while (getline(in_file_map, line_map)) {

        std::istringstream iss_map(line_map);

        // Declare landmark values and ID
        float landmark_x_f, landmark_y_f;
        int id_i;

        // Read data from current line to values
        iss_map >> landmark_x_f;
        iss_map >> landmark_y_f;
        iss_map >> id_i;

        // Declare single_landmark
        Map::single_landmark_s single_landmark_temp;

        // Set values
        single_landmark_temp.id_i = id_i;
        single_landmark_temp.x_f = landmark_x_f;
        single_landmark_temp.y_f = landmark_y_f;

        // Add to landmark list of map
        map.landmark_list.push_back(single_landmark_temp);
    }
    return true;
}

inline double multiv_prob(double sig_x, double sig_y, double x_obs, double y_obs, double mu_x, double mu_y) {
    // calculate normalization term
    double gauss_norm = 1 / (2 * M_PI * sig_x * sig_y);

    // calculate exponent
    double exponent  = (pow(x_obs - mu_x, 2) / (2 * pow(sig_x, 2)))
               + (pow(y_obs - mu_y, 2) / (2 * pow(sig_y, 2)));

    // calculate weight using normalization terms and exponent
    return gauss_norm * exp(-exponent);
}

#endif  // HELPER_FUNCTIONS_H_
