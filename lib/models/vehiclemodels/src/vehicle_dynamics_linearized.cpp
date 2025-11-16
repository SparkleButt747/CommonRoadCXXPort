#include "models/vehiclemodels/vehicle_dynamics_linearized.hpp"

#include <cassert>
#include <cmath>
#include <vector>

#include "models/acceleration_constraints.hpp"
#include "models/steering_constraints.hpp"

namespace {

// Make orientation lie in [-2*pi, 2*pi]
double make_valid_orientation(double angle)
{
    const double two_pi = 2.0 * M_PI;
    while (angle >  two_pi) angle -= two_pi;
    while (angle < -two_pi) angle += two_pi;
    return angle;
}

/**
 * Interpolates an angle value between two angles according to the minimal value
 * of the absolute difference.
 *
 * x  : value of other dimension to interpolate at
 * x1 : lower bound of the other dimension
 * x2 : upper bound of the other dimension
 * y1 : lower bound of angle to interpolate
 * y2 : upper bound of angle to interpolate
 */
double interpolate_angle(double x, double x1, double x2, double y1, double y2)
{
    const double delta = y2 - y1;
    const double y = delta * (x - x1) / (x2 - x1) + y1;
    return make_valid_orientation(y);
}

} // anonymous namespace

namespace velox::models {

std::vector<double> vehicle_dynamics_linearized(const std::vector<double>& x,
                                                const std::vector<double>& u_init,
                                                const VehicleParameters& p,
                                                const std::vector<double>& ref_pos,
                                                const std::vector<double>& ref_theta)
{
    assert(ref_pos.size() == ref_theta.size());
    const std::size_t n = ref_pos.size();
    assert(n >= 2);

    // input constraints
    std::vector<double> u(2);
    u[0] = utils::jerk_dot_constraints(u_init[0], x[3], p.longitudinal);
    u[1] = utils::kappa_dot_dot_constraints(u_init[1], x[7], p.steering);

    // longitudinal and lateral state vector
    const double s = x[0]; // s
    const double v = x[1]; // v

    // longitudinal dynamics
    std::vector<double> f_long(4);
    f_long[0] = x[1]; // s_dot = v
    f_long[1] = x[2]; // v_dot = a
    f_long[2] = x[3]; // a_dot = j
    f_long[3] = u[0]; // j_dot

    // interpolate theta_s from ref_theta at s using same semantics as numpy code
    // Python: s_idx = np.argmax(ref_pos > s) - 1
    std::size_t first_greater = n; // sentinel "not found"
    for (std::size_t i = 0; i < n; ++i) {
        if (ref_pos[i] > s) {
            first_greater = i;
            break;
        }
    }

    std::size_t s_idx;
    if (first_greater == n || first_greater == 0) {
        // mimic s_idx = -1 (wrap to last element)
        s_idx = n - 1;
    } else {
        s_idx = first_greater - 1;
    }
    const std::size_t s_idx_next = (s_idx + 1) % n;

    const double theta_s = interpolate_angle(
        s,
        ref_pos[s_idx],     ref_pos[s_idx_next],
        ref_theta[s_idx],   ref_theta[s_idx_next]
    );

    // lateral dynamics
    std::vector<double> f_lat(4);
    // x5 = d, x6 = theta, x7 = kappa, x8 = kappa_dot
    f_lat[0] = v * x[5] - v * theta_s; // d_dot
    f_lat[1] = v * x[6];               // theta_dot
    f_lat[2] = x[7];                   // kappa_dot
    f_lat[3] = u[1];                   // kappa_dot_dot

    // combine longitudinal and lateral parts
    std::vector<double> f(8);
    for (int i = 0; i < 4; ++i) f[i]     = f_long[i];
    for (int i = 0; i < 4; ++i) f[4 + i] = f_lat[i];

    return f;
}

} // namespace velox::models
