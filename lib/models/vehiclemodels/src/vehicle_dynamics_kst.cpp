#include "models/vehiclemodels/vehicle_dynamics_kst.hpp"

#include <cmath>

#include "common/errors.hpp"

#include "models/acceleration_constraints.hpp"
#include "models/steering_constraints.hpp"

namespace velox::models {

std::vector<double> vehicle_dynamics_kst(const std::vector<double>& x,
                                         const std::vector<double>& u_init,
                                         const VehicleParameters& p)
{
    if (x.size() != 6 || u_init.size() != 2) {
        throw ::velox::errors::SimulationError(
            VELOX_LOC("vehicle_dynamics_kst: expected x.size()==6 and u_init.size()==2"));
    }

    // create equivalent kinematic single-track parameters
    const double l_wb  = p.a + p.b;          // wheel base
    const double l_wbt = p.trailer.l_wb;     // wheel base trailer

    // steering & acceleration constraints
    std::vector<double> u(2);
    u[0] = utils::steering_constraints(x[2], u_init[0], p.steering);
    u[1] = utils::acceleration_constraints(x[3], u_init[1], p.longitudinal);

    // hitch angle constraints
    double d_alpha;
    if (x[5] >= -M_PI / 2.0 && x[5] <= M_PI / 2.0) {
        d_alpha = -x[3] * (std::sin(x[5]) / l_wbt + std::tan(x[2]) / l_wb);
    } else {
        d_alpha = 0.0;
        // Note: Python mutates x[5] here, but for an ODE RHS this has no effect outside;
        // we only need d_alpha = 0 when |x[5]| > pi/2.
    }

    // system dynamics
    std::vector<double> f(6);
    f[0] = x[3] * std::cos(x[4]);
    f[1] = x[3] * std::sin(x[4]);
    f[2] = u[0];
    f[3] = u[1];
    f[4] = x[3] / l_wb * std::tan(x[2]);
    f[5] = d_alpha;

    return f;
}

} // namespace velox::models
