#pragma once

#include <vector>

namespace velox::models {

/**
 * init_ks - generates the initial state vector for the kinematic single-track model.
 *
 * States:
 *  x1 = x-position in a global coordinate system
 *  x2 = y-position in a global coordinate system
 *  x3 = steering angle of front wheels
 *  x4 = velocity in x-direction
 *  x5 = yaw angle
 *
 * u1 = steering angle velocity of front wheels
 * u2 = longitudinal acceleration
 */
std::vector<double> init_ks(const std::vector<double>& init_state);

}  // namespace velox::models
