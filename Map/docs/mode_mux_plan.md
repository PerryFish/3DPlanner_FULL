# Temporary Mode Mux Plan

`bimodal_mode_mux_node` is a temporary integration test layer. It is not the real control state machine and it does not publish `/cmd_vel`, `/mavros/*`, `/fmu/*`, `/actuator/*`, `/offboard_control_mode`, or `/trajectory_setpoint`.

The mux forwards Air outputs when `/bimodal/active_mode` is `AIR`, Ground outputs when it is `GROUND`, and holds output when it is `IDLE`. `simple_mode_commander_node` is only a simulation helper for automatic switching.

Later, a real state machine can subscribe to `/bimodal/active_goal` and `/bimodal/active_path`, or replace this mux entirely.
