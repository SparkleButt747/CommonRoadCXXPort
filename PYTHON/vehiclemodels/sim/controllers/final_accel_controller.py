"""Final longitudinal acceleration controller."""
from __future__ import annotations

from dataclasses import dataclass
import logging

from vehiclemodels.sim.actuators.aero import AeroConfig, AeroModel
from vehiclemodels.sim.actuators.brake import BrakeConfig, BrakeController
from vehiclemodels.sim.actuators.powertrain import Powertrain, PowertrainConfig
from vehiclemodels.sim.actuators.rolling_resistance import (
    RollingResistance,
    RollingResistanceConfig,
)

LOGGER = logging.getLogger(__name__)


@dataclass
class FinalAccelControllerConfig:
    tau_throttle: float
    tau_brake: float
    accel_min: float
    accel_max: float
    stop_speed_epsilon: float = 0.05


@dataclass
class DriverIntent:
    throttle: float
    brake: float


@dataclass
class ControllerOutput:
    acceleration: float
    throttle: float
    brake: float
    drive_force: float
    brake_force: float
    regen_force: float
    hydraulic_force: float
    drag_force: float
    rolling_force: float


class FinalAccelController:
    def __init__(
        self,
        vehicle_mass: float,
        wheel_radius: float,
        powertrain_cfg: PowertrainConfig,
        aero_cfg: AeroConfig,
        rolling_cfg: RollingResistanceConfig,
        brake_cfg: BrakeConfig,
        controller_cfg: FinalAccelControllerConfig,
    ) -> None:
        if vehicle_mass <= 0:
            raise ValueError("vehicle_mass must be positive")
        if wheel_radius <= 0:
            raise ValueError("wheel_radius must be positive")

        self.mass = vehicle_mass
        self.wheel_radius = wheel_radius
        self.powertrain = Powertrain(powertrain_cfg, wheel_radius)
        self.aero = AeroModel(aero_cfg)
        self.rolling = RollingResistance(rolling_cfg)
        self.brakes = BrakeController(brake_cfg)
        self.cfg = controller_cfg

        self.throttle = 0.0
        self.brake = 0.0

    # ------------------------------------------------------------------
    def reset(self) -> None:
        """Reset actuator integrators and the powertrain state."""

        self.throttle = 0.0
        self.brake = 0.0
        self.powertrain.reset()

    # ------------------------------------------------------------------
    def _apply_actuator_dynamics(self, intent: DriverIntent, dt: float) -> None:
        tau_throttle = max(self.cfg.tau_throttle, 1e-3)
        tau_brake = max(self.cfg.tau_brake, 1e-3)
        throttle_target = min(max(intent.throttle, 0.0), 1.0)
        brake_target = min(max(intent.brake, 0.0), 1.0)

        self.throttle += dt / tau_throttle * (throttle_target - self.throttle)
        self.brake += dt / tau_brake * (brake_target - self.brake)

        self.throttle = min(max(self.throttle, 0.0), 1.0)
        self.brake = min(max(self.brake, 0.0), 1.0)

    # ------------------------------------------------------------------
    def step(self, intent: DriverIntent, speed: float, dt: float) -> ControllerOutput:
        self._apply_actuator_dynamics(intent, dt)

        throttle = self.throttle * (1.0 - min(self.brake, 1.0))
        available_regen = self.powertrain.available_regen_torque(speed) / self.wheel_radius
        regen_request, hydraulic_force, _ = self.brakes.blend(self.brake, speed, available_regen)

        regen_torque_request = regen_request * self.wheel_radius
        total_torque, drive_torque, regen_torque = self.powertrain.step(throttle, regen_torque_request, speed, dt)

        drive_force = drive_torque / self.wheel_radius
        regen_force = regen_torque / self.wheel_radius

        if regen_force > regen_request + 1e-6:
            regen_force = regen_request
        hydraulic_force = max(0.0, hydraulic_force + (regen_request - regen_force))
        brake_force = hydraulic_force + regen_force

        drag_force = self.aero.drag_force(speed)
        downforce = self.aero.downforce(speed)
        normal_force = self.mass * 9.81 + downforce
        rolling_force = self.rolling.force(speed, normal_force)

        net_force = drive_force - brake_force + drag_force + rolling_force
        acceleration = net_force / self.mass
        acceleration = max(self.cfg.accel_min, min(self.cfg.accel_max, acceleration))

        if abs(speed) <= self.cfg.stop_speed_epsilon and acceleration < 0.0:
            acceleration = 0.0

        LOGGER.debug(
            "AccelCtrl | v=%.2f m/s throttle=%.2f brake=%.2f drive=%.1fN regen=%.1fN hyd=%.1fN drag=%.1fN roll=%.1fN -> a=%.2fm/s^2",
            speed,
            throttle,
            self.brake,
            drive_force,
            regen_force,
            hydraulic_force,
            drag_force,
            rolling_force,
            acceleration,
        )

        return ControllerOutput(
            acceleration=acceleration,
            throttle=throttle,
            brake=self.brake,
            drive_force=drive_force,
            brake_force=brake_force,
            regen_force=regen_force,
            hydraulic_force=hydraulic_force,
            drag_force=drag_force,
            rolling_force=rolling_force,
        )
