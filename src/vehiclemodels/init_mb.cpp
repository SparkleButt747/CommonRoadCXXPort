#include "vehiclemodels/init_mb.hpp"

#include <cmath>
#include <vector>

#include "vehicle_parameters.hpp"  // for VehicleParameters

namespace vehiclemodels {

std::vector<double> init_mb(const std::vector<double>& init_state,
                            const VehicleParameters& p)
{
    // obtain initial states from vector
    const double sx0     = init_state[0];
    const double sy0     = init_state[1];
    const double delta0  = init_state[2];
    const double vel0    = init_state[3];
    const double Psi0    = init_state[4];
    const double dotPsi0 = init_state[5];
    const double beta0   = init_state[6];

    // create equivalent bicycle parameters
    const double g = 9.81;  // [m/s^2]

    // auxiliary initial states
    const double F0_z_f = p.m_s * g * p.b / (p.a + p.b) + p.m_uf * g;
    const double F0_z_r = p.m_s * g * p.a / (p.a + p.b) + p.m_ur * g;

    std::vector<double> x0;
    x0.reserve(29);

    // sprung mass states
    x0.push_back(sx0);                                // x-position in a global coordinate system
    x0.push_back(sy0);                                // y-position in a global coordinate system
    x0.push_back(delta0);                             // steering angle of front wheels
    x0.push_back(std::cos(beta0) * vel0);             // velocity in x-direction
    x0.push_back(Psi0);                               // yaw angle
    x0.push_back(dotPsi0);                            // yaw rate
    x0.push_back(0.0);                                // roll angle
    x0.push_back(0.0);                                // roll rate
    x0.push_back(0.0);                                // pitch angle
    x0.push_back(0.0);                                // pitch rate
    x0.push_back(std::sin(beta0) * vel0);             // velocity in y-direction
    x0.push_back(0.0);                                // z-position (zero height corresponds to steady state solution)
    x0.push_back(0.0);                                // velocity in z-direction

    // unsprung mass states (front)
    x0.push_back(0.0);                                // roll angle front
    x0.push_back(0.0);                                // roll rate front
    x0.push_back(std::sin(beta0) * vel0 + p.a * dotPsi0); // velocity in y-direction front
    x0.push_back(F0_z_f / (2.0 * p.K_zt));            // z-position front
    x0.push_back(0.0);                                // velocity in z-direction front

    // unsprung mass states (rear)
    x0.push_back(0.0);                                // roll angle rear
    x0.push_back(0.0);                                // roll rate rear
    x0.push_back(std::sin(beta0) * vel0 - p.b * dotPsi0); // velocity in y-direction rear
    x0.push_back(F0_z_r / (2.0 * p.K_zt));            // z-position rear
    x0.push_back(0.0);                                // velocity in z-direction rear

    // wheel states (based on x-velocity / R_w)
    const double omega0 = x0[3] / p.R_w;
    x0.push_back(omega0); // left front wheel angular speed
    x0.push_back(omega0); // right front wheel angular speed
    x0.push_back(omega0); // left rear wheel angular speed
    x0.push_back(omega0); // right rear wheel angular speed

    x0.push_back(0.0);    // delta_y_f
    x0.push_back(0.0);    // delta_y_r

    return x0;
}

}  // namespace vehiclemodels
