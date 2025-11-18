#include <algorithm>
#include <cmath>
#include <iostream>
#include <string_view>

#include "simulation/simulation_daemon.hpp"

namespace vsim = velox::simulation;

struct RunSummary {
    double max_yaw_rate{0.0};
    double max_slip_angle{0.0};
    double max_speed{0.0};
    bool   drift_mode{false};
    bool   low_speed_latched{false};
};

RunSummary run_oversteer_pass(vsim::SimulationDaemon& daemon, bool drift_enabled, std::string_view label)
{
    vsim::ResetParams reset{};
    reset.dt            = 0.02;
    reset.drift_enabled = drift_enabled;
    daemon.reset(reset);

    RunSummary summary{};
    summary.drift_mode = drift_enabled;

    double timestamp = 0.0;
    for (int i = 0; i < 300; ++i) {
        vsim::UserInput input{};
        input.timestamp             = timestamp;
        input.dt                    = 0.02;
        input.longitudinal.throttle = (i < 200) ? 0.25 : 0.05;
        input.steering_nudge        = 1.0;

        const auto telemetry = daemon.step(input);
        summary.max_yaw_rate  = std::max(summary.max_yaw_rate, std::abs(telemetry.velocity.yaw_rate));
        summary.max_slip_angle = std::max(summary.max_slip_angle, std::abs(telemetry.traction.slip_angle));
        summary.max_speed     = std::max(summary.max_speed, telemetry.velocity.speed);
        summary.low_speed_latched = summary.low_speed_latched || telemetry.low_speed_engaged;
        timestamp += input.dt;
    }

    const auto& telemetry = daemon.telemetry();
    std::cout << label << " drift=" << (telemetry.traction.drift_mode ? "on" : "off")
              << " max_speed=" << summary.max_speed
              << " max_yaw=" << summary.max_yaw_rate
              << " max_slip=" << summary.max_slip_angle
              << " latch=" << (summary.low_speed_latched ? "engaged" : "released") << '\n';

    return summary;
}

int main()
{
    vsim::SimulationDaemon::InitParams init{};
    init.model      = vsim::ModelType::STD;
    init.vehicle_id = 1;

    vsim::SimulationDaemon daemon(init);

    const auto safety_run = run_oversteer_pass(daemon, false, "[low-speed safety]");
    const auto drift_run  = run_oversteer_pass(daemon, true, "[drift profile]");

    if (drift_run.max_yaw_rate <= safety_run.max_yaw_rate ||
        drift_run.max_slip_angle <= safety_run.max_slip_angle) {
        std::cerr << "Drift mode did not increase yaw/slip allowance" << std::endl;
        return 1;
    }

    std::cout << "Drift mode demo complete: increased yaw/slip allowances confirmed." << std::endl;
    return 0;
}
