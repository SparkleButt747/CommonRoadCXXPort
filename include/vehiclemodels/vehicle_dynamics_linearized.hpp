#pragma once

#include <vector>
#include "vehicle_parameters.hpp"

namespace vehiclemodels {

/**
 * vehicle_dynamics_linearized - linearized kinematic single-track model:
 * separation of longitudinal and lateral motion.
 *
 * See:
 *   Pek, C. and Althoff, M., "Computationally Efficient Fail-safe Trajectory
 *   Planning for Self-driving Vehicles Using Convex Optimization", ITSC, 2018.
 *
 * State x (combined longitudinal and lateral):
 *   x1 = s        : longitudinal position along reference
 *   x2 = v        : longitudinal velocity
 *   x3 = a        : longitudinal acceleration
 *   x4 = j        : jerk
 *   x5 = d        : lateral deviation from reference
 *   x6 = theta    : global orientation
 *   x7 = kappa    : curvature
 *   x8 = kappa_dot: curvature rate
 *
 * Input u_init:
 *   u1 = j_dot         : change of jerk
 *   u2 = kappa_dot_dot : curvature rate rate
 *
 * ref_pos: reference path position samples (s coordinate)
 * ref_theta: reference path orientation samples (theta), same length as ref_pos
 *
 * Returns:
 *   f = right-hand side of differential equations
 */
std::vector<double> vehicle_dynamics_linearized(const std::vector<double>& x,
                                                const std::vector<double>& u_init,
                                                const VehicleParameters& p,
                                                const std::vector<double>& ref_pos,
                                                const std::vector<double>& ref_theta);

} // namespace vehiclemodels
