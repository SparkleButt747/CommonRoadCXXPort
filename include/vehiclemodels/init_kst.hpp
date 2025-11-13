#pragma once

#include <vector>

namespace vehiclemodels {

/**
 * init_kst - generates the initial state vector for the
 * kinematic single-track model with on-axle trailer.
 *
 * States:
 *  x1 = x-position in a global coordinate system
 *  x2 = y-position in a global coordinate system
 *  x3 = steering angle of front wheels
 *  x4 = velocity in x-direction
 *  x5 = yaw angle
 *  x6 = hitch angle
 */
std::vector<double> init_kst(const std::vector<double>& init_state,
                             double alpha0);

}  // namespace vehiclemodels
