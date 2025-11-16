// tests/scenario_simulator.cpp
// -----------------------------------------------------------------------------
// Replay single-track scenarios from a shared input trace and log state/derivative
// histories. The resulting CSV can be compared against the Python implementation
// to ensure the port stays numerically aligned.
// -----------------------------------------------------------------------------

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "vehicle/parameters_vehicle2.hpp"
#include "models/vehiclemodels/init_st.hpp"
#include "models/vehiclemodels/vehicle_dynamics_st.hpp"

namespace vm = velox::models;

using State = std::vector<double>;
using Input = std::vector<double>;

struct InputSample {
    double time;
    double steering_rate;
    double acceleration;
};

using TraceMap = std::map<std::string, std::vector<InputSample>>;

namespace {

TraceMap load_trace(const std::filesystem::path& path)
{
    std::ifstream in(path);
    if (!in) {
        throw std::runtime_error("scenario_simulator: failed to open trace file: " + path.string());
    }

    std::string header;
    std::getline(in, header); // discard header

    TraceMap traces;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty()) {
            continue;
        }

        std::stringstream ss(line);
        std::string token;

        std::string scenario;
        if (!std::getline(ss, scenario, ',')) {
            throw std::runtime_error("scenario_simulator: malformed line (scenario)");
        }

        if (!std::getline(ss, token, ',')) {
            throw std::runtime_error("scenario_simulator: malformed line (time)");
        }
        double time = std::stod(token);

        if (!std::getline(ss, token, ',')) {
            throw std::runtime_error("scenario_simulator: malformed line (steering_rate)");
        }
        double steering_rate = std::stod(token);

        if (!std::getline(ss, token, ',')) {
            throw std::runtime_error("scenario_simulator: malformed line (acceleration)");
        }
        double acceleration = std::stod(token);

        traces[scenario].push_back({time, steering_rate, acceleration});
    }

    if (traces.empty()) {
        throw std::runtime_error("scenario_simulator: no samples in trace file");
    }

    return traces;
}

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
    std::vector<Input> inputs;
    std::vector<State> derivatives;
};

SimulationResult simulate_trace(const std::string& label,
                                const std::vector<InputSample>& samples,
                                const State& x0,
                                const vm::VehicleParameters& params)
{
    if (samples.empty()) {
        throw std::runtime_error("scenario_simulator: scenario '" + label + "' has no samples");
    }

    SimulationResult result;
    result.label = label;
    result.time.reserve(samples.size());
    result.states.reserve(samples.size());
    result.inputs.reserve(samples.size());
    result.derivatives.reserve(samples.size());

    State x = x0;

    for (std::size_t i = 0; i < samples.size(); ++i) {
        const auto& sample = samples[i];
        Input u{sample.steering_rate, sample.acceleration};

        result.time.push_back(sample.time);
        result.states.push_back(x);
        result.inputs.push_back(u);
        result.derivatives.push_back(vm::vehicle_dynamics_st(x, u, params));

        double dt = 0.0;
        if (i + 1 < samples.size()) {
            dt = samples[i + 1].time - sample.time;
        } else if (i > 0) {
            dt = sample.time - samples[i - 1].time;
        }

        if (dt <= 0.0) {
            throw std::runtime_error("scenario_simulator: non-positive dt encountered");
        }

        x = rk4_step(x, u, params, dt);
    }

    return result;
}

void write_csv(const std::filesystem::path& path,
               const std::vector<SimulationResult>& results)
{
    std::filesystem::create_directories(path.parent_path());

    std::ofstream out(path);
    if (!out) {
        throw std::runtime_error("scenario_simulator: failed to open output file: " + path.string());
    }

    out << "scenario,time,steering_rate,acceleration,";
    out << "sx,sy,delta,velocity,psi,dot_psi,beta,";
    out << "dsx,dsy,ddelta,dvelocity,dpsi,ddot_psi,dbeta\n";
    out.setf(std::ios::fixed, std::ios::floatfield);
    out << std::setprecision(10);

    for (const auto& result : results) {
        for (std::size_t idx = 0; idx < result.time.size(); ++idx) {
            const State& state = result.states[idx];
            const Input& input = result.inputs[idx];
            const State& deriv = result.derivatives[idx];

            out << result.label << ','
                << result.time[idx] << ','
                << input[0] << ','
                << input[1] << ','
                << state[0] << ','
                << state[1] << ','
                << state[2] << ','
                << state[3] << ','
                << state[4] << ','
                << state[5] << ','
                << state[6] << ','
                << deriv[0] << ','
                << deriv[1] << ','
                << deriv[2] << ','
                << deriv[3] << ','
                << deriv[4] << ','
                << deriv[5] << ','
                << deriv[6] << '\n';
        }
    }
}

} // namespace

int main(int argc, char** argv)
{
    try {
        std::filesystem::path input_path = "tests/data/single_track_trace.csv";
        std::filesystem::path output_path = "tests/output/single_track_trace_cpp.csv";

        if (argc >= 2) {
            input_path = argv[1];
        }
        if (argc >= 3) {
            output_path = argv[2];
        }

        TraceMap traces = load_trace(input_path);

        vm::VehicleParameters params = vm::parameters_vehicle2();
        State initial_state{0.0, 0.0, 0.0, 15.0, 0.0, 0.0, 0.0};
        State x0 = vm::init_st(initial_state);

        std::vector<SimulationResult> results;
        results.reserve(traces.size());

        for (const auto& [label, samples] : traces) {
            results.push_back(simulate_trace(label, samples, x0, params));
        }

        std::sort(results.begin(), results.end(),
                  [](const SimulationResult& a, const SimulationResult& b) {
                      return a.label < b.label;
                  });

        write_csv(output_path, results);

        std::cout << "scenario_simulator: wrote " << output_path << '\n';
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << '\n';
        return 1;
    }

    return 0;
}
