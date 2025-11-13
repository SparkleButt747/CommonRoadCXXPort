#include "vehiclemodels/init_std.hpp"

#include <cmath>
#include <vector>

#include "vehicle_parameters.hpp"  // for VehicleParameters

namespace vehiclemodels {

std::vector<double> init_std(const std::vector<double>& init_state,
                             const VehicleParameters& p)
{
    // create initial state vector
    std::vector<double> x0 = init_state;

    // x0[3] = velocity at vehicle center
    // x0[2] = steering angle (delta)
    // x0[6] = slip angle at vehicle center
    const double v    = x0[3];
    const double beta = x0[6];
    const double delta = x0[2];

    // init front wheel angular speed
    x0.push_back(v * std::cos(beta) * std::cos(delta) / p.R_w);
    // init rear wheel angular speed
    x0.push_back(v * std::cos(beta) / p.R_w);

    return x0;
}

}  // namespace vehiclemodels
