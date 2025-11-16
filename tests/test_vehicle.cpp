// tests/test_vehicle.cpp
// -----------------------------------------------------------------------------
// Numerical experiment mirroring PYTHON/scripts/test_vehicle.py (ST scenario)
// -----------------------------------------------------------------------------

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "vehicle/parameters_vehicle2.hpp"
#include "models/vehiclemodels/init_st.hpp"
#include "models/vehiclemodels/vehicle_dynamics_st.hpp"

namespace vm = velox::models;

using State = std::vector<double>;
using Input = std::vector<double>;

namespace {

State rk4_step(const State& x,
               const Input& u,
               const vm::VehicleParameters& p,
               double dt)
{
    auto dyn = [&](const State& state) {
        return vm::vehicle_dynamics_st(state, u, p);
    };

    State k1 = dyn(x);

    State x_tmp(x.size());
    for (std::size_t i = 0; i < x.size(); ++i) {
        x_tmp[i] = x[i] + 0.5 * dt * k1[i];
    }
    State k2 = dyn(x_tmp);

    for (std::size_t i = 0; i < x.size(); ++i) {
        x_tmp[i] = x[i] + 0.5 * dt * k2[i];
    }
    State k3 = dyn(x_tmp);

    for (std::size_t i = 0; i < x.size(); ++i) {
        x_tmp[i] = x[i] + dt * k3[i];
    }
    State k4 = dyn(x_tmp);

    State x_next(x.size());
    for (std::size_t i = 0; i < x.size(); ++i) {
        x_next[i] = x[i] + (dt / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
    }

    return x_next;
}

struct SimulationResult {
    std::string label;
    std::vector<double> time;
    std::vector<State> states;
};

SimulationResult simulate_case(const std::string& label,
                               const State& x0,
                               const Input& u,
                               const vm::VehicleParameters& p,
                               double t_start,
                               double t_final,
                               double dt)
{
    const int steps = static_cast<int>((t_final - t_start) / dt);

    SimulationResult result;
    result.label = label;
    result.time.reserve(steps);
    result.states.reserve(steps);

    State x = x0;
    double t = t_start;

    for (int i = 0; i < steps; ++i) {
        result.time.push_back(t);
        result.states.push_back(x);
        x = rk4_step(x, u, p, dt);
        t += dt;
    }

    return result;
}

void write_csv(const std::string& path,
               const std::vector<SimulationResult>& cases)
{
    std::ofstream out(path);
    if (!out) {
        throw std::runtime_error("Failed to open output file: " + path);
    }

    out << "scenario,time,sx,sy,delta,velocity,psi,dot_psi,beta\n";
    out.setf(std::ios::fixed, std::ios::floatfield);
    out << std::setprecision(10);

    for (const auto& result : cases) {
        for (std::size_t idx = 0; idx < result.time.size(); ++idx) {
            const State& x = result.states[idx];
            out << result.label << ','
                << result.time[idx] << ','
                << x[0] << ','
                << x[1] << ','
                << x[2] << ','
                << x[3] << ','
                << x[4] << ','
                << x[5] << ','
                << x[6] << '\n';
        }
    }
}

} // namespace

int main()
{
    try {
        const double t_start = 0.0;
        const double t_final = 1.0;
        const double dt = 0.01;

        vm::VehicleParameters p = vm::parameters_vehicle2();
        const double g = 9.81;

        State initial_state{0.0, 0.0, 0.0, 15.0, 0.0, 0.0, 0.0};
        State x0_st = vm::init_st(initial_state);

        std::vector<std::pair<std::string, Input>> cases{
            {"coasting", {0.15, 0.0}},
            {"braking", {0.15, -0.75 * g}},
            {"accelerating", {0.15, 0.63 * g}}
        };

        std::vector<SimulationResult> results;
        results.reserve(cases.size());

        for (const auto& [label, input] : cases) {
            results.push_back(
                simulate_case(label, x0_st, input, p, t_start, t_final, dt)
            );
        }

        const std::string output_dir = "tests/output";
        std::filesystem::create_directories(output_dir);
        const std::string output_path = output_dir + "/oversteer_understeer_ST_cpp.csv";
        write_csv(output_path, results);

        std::cout << "Wrote C++ ST trajectories to " << output_path << '\n';
    } catch (const std::exception& ex) {
        std::cerr << "test_vehicle: " << ex.what() << '\n';
        return 1;
    }

    return 0;
}
