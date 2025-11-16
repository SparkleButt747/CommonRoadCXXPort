#pragma once

#include "models/steering_parameters.hpp"

namespace velox::models {
namespace utils {

/**
 * Adjusts the steering velocity based on steering constraints.
 */
double steering_constraints(double steering_angle,
                            double steering_velocity,
                            const SteeringParameters& p);

/**
 * Adjusts kappa_dot_dot if curvature-rate limits or input bounds are reached.
 */
double kappa_dot_dot_constraints(double kappa_dot_dot,
                                 double kappa_dot,
                                 const SteeringParameters& p);

} // namespace utils
} // namespace velox::models
