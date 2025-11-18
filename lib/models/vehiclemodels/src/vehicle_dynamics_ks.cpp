#include "models/vehiclemodels/vehicle_dynamics_ks.hpp"

#include <cmath>

#include "common/errors.hpp"

#include "models/acceleration_constraints.hpp"
#include "models/steering_constraints.hpp"

namespace velox::models {

std::vector<double> vehicle_dynamics_ks(const std::vector<double>& x,
                                        const std::vector<double>& u_init,
                                        const VehicleParameters& p)
{
    if (x.size() != 5 || u_init.size() != 2) {
        throw ::velox::errors::SimulationError(
            VELOX_LOC("vehicle_dynamics_ks: expected x.size()==5 and u_init.size()==2"));
    }

    // create equivalent kinematic single-track parameters
    const double l = p.a + p.b;

    // consider steering constraints
    std::vector<double> u(2);
    u[0] = utils::steering_constraints(x[2], u_init[0], p.steering);
    // consider acceleration constraints
    u[1] = utils::acceleration_constraints(x[3], u_init[1], p.longitudinal);

    // system dynamics
    std::vector<double> f(5);
    f[0] = x[3] * std::cos(x[4]);
    f[1] = x[3] * std::sin(x[4]);
    f[2] = u[0];
    f[3] = u[1];
    f[4] = x[3] / l * std::tan(x[2]);

    return f;
}

} // namespace velox::models
