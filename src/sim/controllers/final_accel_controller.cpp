#include "sim/longitudinal/final_accel_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace vehiclemodels::sim::longitudinal {

namespace {
constexpr double kGravity = 9.81;
constexpr double kEpsilon = 1e-6;
}

FinalAccelController::FinalAccelController(double vehicle_mass,
                                           double wheel_radius,
                                           PowertrainConfig powertrain_cfg,
                                           AeroConfig aero_cfg,
                                           RollingResistanceConfig rolling_cfg,
                                           BrakeConfig brake_cfg,
                                           FinalAccelControllerConfig controller_cfg)
    : mass_(vehicle_mass)
    , wheel_radius_(wheel_radius)
    , powertrain_(std::move(powertrain_cfg), wheel_radius)
    , aero_(std::move(aero_cfg))
    , rolling_(std::move(rolling_cfg))
    , brakes_(std::move(brake_cfg))
    , cfg_(controller_cfg)
{
    if (!std::isfinite(mass_) || mass_ <= 0.0) {
        throw std::invalid_argument("vehicle_mass must be positive and finite");
    }
    if (!std::isfinite(wheel_radius_) || wheel_radius_ <= 0.0) {
        throw std::invalid_argument("wheel_radius must be positive and finite");
    }
    cfg_.validate();
}

void FinalAccelController::reset()
{
    throttle_ = 0.0;
    brake_    = 0.0;
    powertrain_.reset();
}

void FinalAccelController::apply_actuator_dynamics(const DriverIntent& intent, double dt)
{
    const double tau_throttle = std::max(cfg_.tau_throttle, 1e-3);
    const double tau_brake    = std::max(cfg_.tau_brake, 1e-3);

    const double throttle_target = std::clamp(intent.throttle, 0.0, 1.0);
    const double brake_target    = std::clamp(intent.brake, 0.0, 1.0);

    throttle_ += dt / tau_throttle * (throttle_target - throttle_);
    brake_ += dt / tau_brake * (brake_target - brake_);

    throttle_ = std::clamp(throttle_, 0.0, 1.0);
    brake_    = std::clamp(brake_, 0.0, 1.0);
}

ControllerOutput FinalAccelController::step(const DriverIntent& intent, double speed, double dt)
{
    apply_actuator_dynamics(intent, dt);

    const double throttle_command = throttle_ * (1.0 - std::min(brake_, 1.0));
    const double available_regen_force = powertrain_.available_regen_torque(speed) / wheel_radius_;
    const BrakeBlendOutput brake_output =
        brakes_.blend(brake_, speed, available_regen_force);

    const double regen_torque_request = brake_output.regen_force * wheel_radius_;
    const PowertrainOutput powertrain_output =
        powertrain_.step(throttle_command, regen_torque_request, speed, dt);

    double drive_force = powertrain_output.drive_torque / wheel_radius_;
    double regen_force = powertrain_output.regen_torque / wheel_radius_;

    if (regen_force > brake_output.regen_force + kEpsilon) {
        regen_force = brake_output.regen_force;
    }
    double hydraulic_force =
        std::max(0.0, brake_output.hydraulic_force + (brake_output.regen_force - regen_force));
    const double brake_force = hydraulic_force + regen_force;

    const double drag_force    = aero_.drag_force(speed);
    const double downforce     = aero_.downforce(speed);
    const double normal_force  = mass_ * kGravity + downforce;
    const double rolling_force = rolling_.force(speed, normal_force);

    const double net_force = drive_force - brake_force + drag_force + rolling_force;
    double acceleration    = net_force / mass_;
    acceleration           = std::clamp(acceleration, cfg_.accel_min, cfg_.accel_max);

    if (std::abs(speed) <= cfg_.stop_speed_epsilon && acceleration < 0.0) {
        acceleration = 0.0;
    }

    return ControllerOutput{
        .acceleration    = acceleration,
        .throttle        = throttle_command,
        .brake           = brake_,
        .drive_force     = drive_force,
        .brake_force     = brake_force,
        .regen_force     = regen_force,
        .hydraulic_force = hydraulic_force,
        .drag_force      = drag_force,
        .rolling_force   = rolling_force,
    };
}

} // namespace vehiclemodels::sim::longitudinal
