#include "sim/vehicle_simulator.hpp"

#include <stdexcept>

namespace vehiclemodels::sim {

VehicleSimulator::VehicleSimulator(ModelInterface model,
                                   VehicleParameters params,
                                   double dt,
                                   LowSpeedSafety safety)
    : model_(std::move(model))
    , params_(std::move(params))
    , safety_(std::move(safety))
{
    if (!model_.valid()) {
        throw std::invalid_argument("VehicleSimulator requires a valid ModelInterface");
    }
    set_dt(dt);
    safety_.reset();
}

void VehicleSimulator::reset(const std::vector<double>& initial_state)
{
    state_ = initial_state;
    safety_.reset();
    apply_safety(state_, true);
}

double VehicleSimulator::speed() const
{
    ensure_ready();
    return model_.speed_fn(state_, params_);
}

const std::vector<double>& VehicleSimulator::step(const std::vector<double>& control)
{
    ensure_ready();
    if (control.size() != 2) {
        throw std::invalid_argument("VehicleSimulator control must contain steering rate and acceleration");
    }

    const double dt = dt_;
    const auto& current_state = state_;

    auto [k1, current] = dynamics(current_state, control, true);
    state_ = current;

    auto k2_state = add_scaled(state_, 0.5 * dt, k1);
    auto [k2, _k2_state] = dynamics(k2_state, control, false);
    (void)_k2_state;

    auto k3_state = add_scaled(state_, 0.5 * dt, k2);
    auto [k3, _k3_state] = dynamics(k3_state, control, false);
    (void)_k3_state;

    auto k4_state = add_scaled(state_, dt, k3);
    auto [k4, _k4_state] = dynamics(k4_state, control, false);
    (void)_k4_state;

    if (k1.size() != state_.size() || k2.size() != state_.size() ||
        k3.size() != state_.size() || k4.size() != state_.size()) {
        throw std::runtime_error("VehicleSimulator dynamics returned mismatched state dimension");
    }

    for (std::size_t i = 0; i < state_.size(); ++i) {
        state_[i] += (dt / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
    }

    apply_safety(state_, true);
    return state_;
}

void VehicleSimulator::set_dt(double dt)
{
    if (!(dt > 0.0)) {
        throw std::invalid_argument("VehicleSimulator timestep must be positive");
    }
    dt_ = dt;
}

void VehicleSimulator::ensure_ready() const
{
    if (state_.empty()) {
        throw std::runtime_error("VehicleSimulator has not been initialised; call reset() first");
    }
}

std::vector<double> VehicleSimulator::add_scaled(const std::vector<double>& base,
                                                 double scale,
                                                 const std::vector<double>& delta) const
{
    if (base.size() != delta.size()) {
        throw std::runtime_error("VehicleSimulator::add_scaled size mismatch");
    }
    std::vector<double> result(base.size());
    for (std::size_t i = 0; i < base.size(); ++i) {
        result[i] = base[i] + scale * delta[i];
    }
    return result;
}

std::pair<std::vector<double>, std::vector<double>> VehicleSimulator::dynamics(const std::vector<double>& state,
                                                                               const std::vector<double>& control,
                                                                               bool update_latch)
{
    std::vector<double> sanitized = state;
    apply_safety(sanitized, update_latch);
    auto rhs = model_.dynamics_fn(sanitized, control, params_);
    return {std::move(rhs), std::move(sanitized)};
}

void VehicleSimulator::apply_safety(std::vector<double>& state, bool update_latch)
{
    const double speed = model_.speed_fn(state, params_);
    safety_.apply(state, speed, update_latch);
}

} // namespace vehiclemodels::sim
