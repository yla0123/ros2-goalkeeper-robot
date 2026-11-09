# Goalkeeper Bringup

The active simulation entry point is `ros2 launch goalkeeper_sim simulation.launch.py`. It loads the generated URDF/world, bridges the native Gazebo controller and feedback, and starts the checked command adapter. Optional `shoot:=true shot_case:=low` starts one scripted shot after measured standby.

See the [runtime steps](../../docs/robot-design.md#run-and-check) and [active interfaces](../../docs/interfaces.md). A separate hardware launch is not implemented.
