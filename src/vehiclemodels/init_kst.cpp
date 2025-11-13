#include "vehiclemodels/init_kst.hpp"

namespace vehiclemodels {

std::vector<double> init_kst(const std::vector<double>& init_state,
                             double alpha0)
{
    // obtain initial states from vector
    const double sx0    = init_state[0];
    const double sy0    = init_state[1];
    const double delta0 = init_state[2];
    const double vel0   = init_state[3];
    const double Psi0   = init_state[4];

    std::vector<double> x0;
    x0.reserve(6);
    x0.push_back(sx0);
    x0.push_back(sy0);
    x0.push_back(delta0);
    x0.push_back(vel0);
    x0.push_back(Psi0);
    x0.push_back(alpha0); // hitch angle

    return x0;
}

}  // namespace vehiclemodels
