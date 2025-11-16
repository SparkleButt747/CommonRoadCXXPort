#include "models/vehiclemodels/init_ks.hpp"

namespace velox::models {

std::vector<double> init_ks(const std::vector<double>& init_state)
{
    // obtain initial states from vector
    const double sx0    = init_state[0];
    const double sy0    = init_state[1];
    const double delta0 = init_state[2];
    const double vel0   = init_state[3];
    const double Psi0   = init_state[4];

    // sprung mass states
    std::vector<double> x0;
    x0.reserve(5);
    x0.push_back(sx0);    // x-position in a global coordinate system
    x0.push_back(sy0);    // y-position in a global coordinate system
    x0.push_back(delta0); // steering angle of front wheels
    x0.push_back(vel0);   // velocity
    x0.push_back(Psi0);   // yaw angle

    return x0;
}

}  // namespace velox::models
