#include "utils/steering_constraints.hpp"

#include <cmath>

namespace vehiclemodels {
namespace utils {

double steering_constraints(double steering_angle,
                            double steering_velocity,
                            const SteeringParameters& p)
{
    // steering limit reached?
    if ((steering_angle <= p.min && steering_velocity <= 0.0) ||
        (steering_angle >= p.max && steering_velocity >= 0.0))
    {
        steering_velocity = 0.0;
    } else if (steering_velocity <= p.v_min) {
        steering_velocity = p.v_min;
    } else if (steering_velocity >= p.v_max) {
        steering_velocity = p.v_max;
    }

    return steering_velocity;
}

double kappa_dot_dot_constraints(double kappa_dot_dot,
                                 double kappa_dot,
                                 const SteeringParameters& p)
{
    // input constraints for kappa_dot_dot
    if ((kappa_dot < -p.kappa_dot_max && kappa_dot_dot < 0.0) ||
        (kappa_dot >  p.kappa_dot_max && kappa_dot_dot > 0.0))
    {
        // kappa_dot limit reached
        kappa_dot_dot = 0.0;
    } else if (std::fabs(kappa_dot_dot) >= p.kappa_dot_dot_max) {
        // input bounds reached
        kappa_dot_dot = p.kappa_dot_dot_max;
    }
    return kappa_dot_dot;
}

} // namespace utils
} // namespace vehiclemodels
