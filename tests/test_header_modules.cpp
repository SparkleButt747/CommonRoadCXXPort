#include "models/acceleration_constraints.hpp"
#include "models/longitudinal_parameters.hpp"
#include "models/steering_constraints.hpp"
#include "models/steering_parameters.hpp"
#include "models/tireParameters.hpp"
#include "models/trailer_parameters.hpp"
#include "models/unitConversion.hpp"

#include <cmath>

using namespace velox::models::utils;

namespace {
bool nearly_equal(double a, double b, double eps = 1e-9)
{
    return std::fabs(a - b) <= eps;
}
}

int main()
{
    LongitudinalParameters longitudinal{};
    longitudinal.v_min    = 0.0;
    longitudinal.v_max    = 40.0;
    longitudinal.v_switch = 10.0;
    longitudinal.a_max    = 3.0;
    longitudinal.j_max    = 2.0;
    longitudinal.j_dot_max = 5.0;

    const double accel = acceleration_constraints(5.0, 10.0, longitudinal);
    if (!nearly_equal(accel, longitudinal.a_max)) {
        return 1;
    }

    const double jerk_dot = jerk_dot_constraints(10.0, 0.0, longitudinal);
    if (!nearly_equal(jerk_dot, longitudinal.j_dot_max)) {
        return 2;
    }

    SteeringParameters steering{};
    steering.min = -0.5;
    steering.max = 0.5;
    steering.v_min = -0.3;
    steering.v_max = 0.3;
    steering.kappa_dot_max = 0.2;
    steering.kappa_dot_dot_max = 0.5;

    const double steering_rate = steering_constraints(0.4, 1.0, steering);
    if (!nearly_equal(steering_rate, steering.v_max)) {
        return 3;
    }

    const double kappa_limited = kappa_dot_dot_constraints(1.0, steering.kappa_dot_max + 0.1, steering);
    if (!nearly_equal(kappa_limited, 0.0)) {
        return 4;
    }

    const double kappa_input = kappa_dot_dot_constraints(steering.kappa_dot_dot_max + 1.0, 0.0, steering);
    if (!nearly_equal(kappa_input, steering.kappa_dot_dot_max)) {
        return 5;
    }

    TrailerParameters trailer{};
    trailer.l = 5.0;
    trailer.w = 2.0;

    TireParameters tire{};
    tire.p_cx1 = 1.0;

    const double converted_length = ft_IN_m(10.0);
    const double converted_force  = lbs_ft_IN_N_m(1.0);
    if (!(converted_length > 0.0 && converted_force > 0.0)) {
        return 6;
    }

    // Use structures to avoid unused warnings.
    const double trailer_extent = trailer.l + trailer.w;
    const double tire_factor    = tire.p_cx1;
    if (!(trailer_extent > 0.0 && tire_factor == 1.0)) {
        return 7;
    }

    return 0;
}
