# MIE1075 Project: Autonomous Goalkeeper Robot

The regulation goal has a **7.56 m × 2.56 m outer frame** and a **7.32 m × 2.44 m clear opening**. The goalkeeper is **2.40 m high when upright**. The chassis slides **±3.55 m** along a path **0.52 m in front of the goal line**, and the board tilts through **180°**. Across the full slide range, the upright model can cover any legal ball centre in front projection, given enough time to position.

![Regulation goal and wheeled keeper dimensions](docs/images/goalkeeper-scale.svg)

## Build and run

Use Ubuntu 24.04 with ROS 2 Jazzy and Gazebo Harmonic. From the repository root:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --base-paths src
source install/setup.bash
ros2 launch goalkeeper_sim simulation.launch.py
```

To see both motions, run the same launch with `demo:=true`:

```bash
ros2 launch goalkeeper_sim simulation.launch.py demo:=true
```

The demo starts at centre, slides and tilts to one side, returns to centre, and repeats on the other side. Each outward move covers 3.55 m and is designed for a 0.50 s movement window. The launch without `demo:=true` waits for a command or shot.

The launch loads the generated world and URDF, a Gazebo joint trajectory controller, state feedback, and the ROS adapter. In a second sourced terminal, check readiness and joint state:

```bash
ros2 topic echo /keeper/ready --once
ros2 topic echo /joint_states --once
```

See [robot design and validation](docs/robot-design.md) for the manual movement sequence and scripted shot runs. For an offline check with Python 3 alone:

```bash
python3 tools/generate_geometry.py
python3 tests/test_regulation_geometry.py -v
python3 run_keeper.py demo --case low --path-csv action_path.csv
```

`config/keeper.design.json` is the active parameter file. Regenerate and rebuild after changing it. See [architecture](docs/architecture.md), [interfaces](docs/interfaces.md), and the [integration plan](docs/implementation-plan.md).
