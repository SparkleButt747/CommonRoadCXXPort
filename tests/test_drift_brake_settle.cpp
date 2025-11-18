#include <cassert>
#include <cmath>
#include <vector>

#include "controllers/longitudinal/final_accel_controller.hpp"
#include "io/config_manager.hpp"
#include "models/vehiclemodels/init_mb.hpp"
#include "models/vehiclemodels/vehicle_dynamics_mb.hpp"
#include "simulation/model_timing.hpp"
#include "simulation/vehicle_simulator.hpp"
#include "vehicle_parameters.hpp"

namespace vio  = velox::io;
namespace vm   = velox::models;
namespace vsim = velox::simulation;
namespace vml  = velox::controllers::longitudinal;

int main()
{
    vio::ConfigManager configs{};

    const auto params      = configs.load_vehicle_parameters(2);
    auto       safety_cfg  = configs.load_low_speed_safety_config(vsim::ModelType::MB);
    safety_cfg.drift_enabled = true;

    vsim::LowSpeedSafety safety(safety_cfg,
                                /*longitudinal_index=*/3,
                                /*lateral_index=*/10,
                                /*yaw_rate_index=*/5,
                                /*slip_index=*/6,
                                /*wheel_speed_indices=*/{23, 24, 25, 26},
                                /*steering_index=*/2,
                                /*wheelbase=*/params.a + params.b,
                                /*rear_length=*/params.b);

    vml::FinalAccelController controller(params.m,
                                         params.R_w,
                                         configs.load_powertrain_config(),
                                         configs.load_aero_config(),
                                         configs.load_rolling_resistance_config(),
                                         configs.load_brake_config(),
                                         configs.load_final_accel_controller_config());

    vsim::ModelInterface model{};
    model.init_fn = [](const std::vector<double>& init_state, const vm::VehicleParameters& p) {
        return vm::init_mb(init_state, p);
    };
    model.dynamics_fn = [](const std::vector<double>& x,
                           const std::vector<double>& u,
                           const vm::VehicleParameters& p,
                           double) {
        return vm::vehicle_dynamics_mb(x, u, p);
    };
    model.speed_fn = [](const std::vector<double>& state, const vm::VehicleParameters&) {
        const double v_long = (state.size() > 3) ? state[3] : 0.0;
        const double v_lat  = (state.size() > 10) ? state[10] : 0.0;
        return std::hypot(v_long, v_lat);
    };

    const double dt = 0.01;
    vsim::VehicleSimulator simulator(model, params, dt, safety);

    simulator.reset({0.0, 0.0, 0.2, 8.0, 0.0, 0.5, 0.0});

    double max_yaw_rate      = 0.0;
    double yaw_before_stop   = 0.0;
    double yaw_at_stop       = 0.0;
    bool   captured_slide    = false;
    bool   engaged_telemetry = false;

    for (int i = 0; i < 2400; ++i) {
        const double speed  = simulator.speed();
        const auto   accel  = controller.step(vml::DriverIntent{0.0, 1.0}, speed, dt);
        const double steer  = 2.0;

        simulator.set_dt(dt);
        simulator.step({steer, accel.acceleration});

        const double yaw_rate = std::abs(simulator.state()[5]);
        max_yaw_rate          = std::max(max_yaw_rate, yaw_rate);
        engaged_telemetry = engaged_telemetry || simulator.safety().engaged();

        if (speed > safety_cfg.drift.release_speed) {
            captured_slide = captured_slide || yaw_rate > 0.3;
        } else if (speed > safety_cfg.stop_speed_epsilon) {
            yaw_before_stop = yaw_rate;
        } else {
            yaw_at_stop = yaw_rate;
            break;
        }
    }

    assert(engaged_telemetry);
    assert(captured_slide);
    assert(yaw_at_stop < yaw_before_stop);
    assert(max_yaw_rate > 0.3);

    // Spin latch should release cleanly once we accelerate again.
    bool released_latch = false;
    simulator.set_dt(dt);
    for (int i = 0; i < 800; ++i) {
        const double speed  = simulator.speed();
        const auto   accel  = controller.step(vml::DriverIntent{0.6, 0.0}, speed, dt);
        simulator.step({0.0, accel.acceleration});
        if (!simulator.safety().engaged() && speed >= safety_cfg.drift.release_speed) {
            released_latch = true;
            break;
        }
    }

    assert(released_latch);

    return 0;
}
