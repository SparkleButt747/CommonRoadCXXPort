#pragma once

#include "models/longitudinal_parameters.hpp"

namespace velox::models {
namespace utils {

/**
 * Adjusts the acceleration based on acceleration constraints.
 */
double acceleration_constraints(double velocity,
                                double acceleration,
                                const LongitudinalParameters& p);

/**
 * Adjusts jerk_dot if jerk limit or input bounds are reached.
 */
double jerk_dot_constraints(double jerk_dot,
                            double jerk,
                            const LongitudinalParameters& p);

} // namespace utils
} // namespace velox::models
