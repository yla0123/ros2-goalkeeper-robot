# Goalkeeper Bringup

Status: proposed module; launch files and ROS 2 package files are not implemented.

Provide manual_sim and autonomous_sim launch modes. Each mode must select one command source and the simulation backend, load consistent configuration, and start disarmed. Keep backend selection separate so a later external controller adapter can use the same target contract.

Changing a mode must stop and disarm before handing over control. The [example configuration](../../config/keeper.example.yaml) supplies simulation and planning settings.

See the [implementation plan](../../docs/implementation-plan.md).
