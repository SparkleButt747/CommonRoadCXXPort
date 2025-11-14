# Low-speed launch with steering lock: diagnostic notes

## Summary
When the vehicle is at rest and both full throttle and maximum steering lock are applied, the STD and MB dynamics never accelerate past ~0.2–0.3 m/s until the steering input is removed. This is caused by the low-speed safety latch zeroing lateral states while the wheels remain heavily steered, leading to unrealistically large lateral tire forces that counteract the engine torque.

## Trigger
1. Bring the vehicle to a complete stop.
2. Apply full steering lock (left or right) and full throttle.
3. Observe that the simulated speed hovers around 0.2–0.3 m/s and does not increase until the steering input is relaxed.

## Root cause
- The low-speed safety latch engages below `engage_speed` and forces the yaw-rate, slip angle, and (for the multi-body model) lateral velocity states to zero while engaged. 【F:PYTHON/vehiclemodels/sim/low_speed_safety.py†L76-L116】
- With the steering angle held at its limit, the single-track drift model computes a large negative front slip angle (≈ `-delta`) because yaw rate and slip are clamped to zero. This produces strong lateral tire forces that appear in the longitudinal acceleration term with a braking sign, counteracting the propulsive forces. 【F:PYTHON/vehiclemodels/vehicle_dynamics_std.py†L66-L157】
- The multi-body model experiences the same issue because its lateral velocity (state index 10) is limited to ±`stop_speed_epsilon` while the yaw rate remains zero, again forcing unrealistic slip angles and lateral drag. 【F:PYTHON/scripts/pygame_vehicle_tester.py†L117-L146】【F:PYTHON/vehiclemodels/vehicle_dynamics_mb.py†L273-L359】

## Suggested direction
Allow the latch to bound yaw rate, slip, and lateral velocity toward a kinematic reference instead of pinning them at zero. For example, when engaged we could project the state onto the kinematic single-track solution for the current steering angle and speed, or relax the clamp to permit the expected yaw rate `v / lwb * tan(delta)` and lateral velocity `v * tan(delta)`. This keeps the safety rails while avoiding the counter-productive lateral forces that trap the vehicle at very low speed.

