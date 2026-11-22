# Implementation Plan

## 1. Repository layout

The repository includes a local planner, a generated human-shaped URDF, a Gazebo launch file, and a ROS 2 shooter node. Interception ROS nodes, custom messages, perception, and joint execution remain to be implemented.

```text
autonomous-goalkeeper-robot/
|-- readme.md
|-- docs/
|   |-- robot-design.md
|   |-- architecture.md
|   |-- interfaces.md
|   |-- implementation-plan.md
|   `-- reuse-decisions.md
|-- config/
|   |-- keeper.design.json
|   `-- keeper.example.yaml
|-- run_keeper.py
|-- tools/generate_geometry.py      Shared-config geometry generator
|-- tests/test_regulation_geometry.py
`-- src/
    |-- goalkeeper_sim/             Existing static world and package scaffold
    |-- goalkeeper_description/     Generated adult-sized slide/tilt model
    |-- goalkeeper_interfaces/      Proposed custom messages
    |-- goalkeeper_control/         Local planner demo; ROS 2 nodes and adapters proposed
    |-- goalkeeper_perception/      Proposed detector, tracker, and predictor
    |-- goalkeeper_bringup/         Proposed control launch and mode configuration
    `-- penalty_shooter/            Existing ROS 2 shooter source
```

## 2. Stages and acceptance checks

| Stage | Work | Acceptance check |
| --- | --- | --- |
| 1. Geometry and interface | Use the regulation goal and adult silhouette in `docs/robot-design.md`; regenerate assets after configuration changes | Goal inside edges and URDF body parts match the planner; gaps remain unoccupied |
| 2. Local planning baseline | Use the Python dry run to inspect coverage and reachability under the example limits | Reachable and unreachable target cases are correctly reported before connecting hardware |
| 3. Two-axis simulation | Connect the generated slide/tilt URDF to a Gazebo control adapter | Slide and tilt targets use the expected units, signs, and limits |
| 4. Manual control | Implement KeeperCommand, command arbitration, and simulation status | Both axes reach repeated targets; invalid or expired commands are rejected |
| 5. Execution adapter contract | Define backend-independent commands, feedback, and failure handling | A mock adapter reports acceptance, completion, timeout, and disconnect outcomes separately |
| 6. Perception baseline | Fix the camera, calibrate its transform, detect a colored ball, estimate velocity | Position estimates are compared with known locations and report invalid data when the ball is lost |
| 7. Autonomous interception | Connect the crossing predictor to the planner | Reachable and unreachable shots are separated; the keeper arrives before the ball in controlled trials |
| 8. Evaluation | Repeat a defined set of shots and record timing, positions, and outcomes | Report trial count, conditions, failures, positioning error, and save rate |

Use the stated geometry and timing assumptions as software acceptance criteria. Do not use commercial product performance as the prototype acceptance threshold.

## 3. Proposed implementation files

| Module | Files to implement later | Main work |
| --- | --- | --- |
| goalkeeper_description | urdf/keeper.urdf.xacro; model configuration | slide_joint along +Y; tilt_joint about +X; collision geometry and limits |
| goalkeeper_interfaces | msg/BallState.msg; msg/InterceptTarget.msg; msg/KeeperCommand.msg; msg/KeeperStatus.msg | Register message definitions and their dependencies |
| goalkeeper_control | local Python planning demo; manual_target_node.py; command_arbiter_node.py; interception_planner_node.py; execution_adapter_node.py; simulation_adapter_node.py | The demo maps crossing coordinates to a candidate pose; ROS 2 nodes and execution adapters remain to be implemented |
| goalkeeper_perception | ball_detector_node.py; tracker_node.py; crossing_predictor_node.py | Detection, transform into goal_frame, velocity estimation, and crossing prediction |
| goalkeeper_bringup | manual_sim.launch.py; autonomous_sim.launch.py; backend configuration | Load consistent configuration and exactly one execution backend |
| goalkeeper_sim | Spawn configuration, ball and camera models, world resource installation | Extend the current static scene; install resources for package-based launch |

The local planner runs through `python3 run_keeper.py demo`; add `--path-csv action_path.csv` for its movement path. The demo uses a built-in coordinate. The Gazebo model and shooter ROS packages have implementation files, while the proposed interception control and perception nodes remain unavailable. Model spawning and end-to-end interception still require ROS/Gazebo runtime validation.

## 4. Integration choices

- Use a fixed camera for the first prototype to simplify coordinate calibration.
- Begin with colored-ball detection and a linear low-shot baseline. Add a learned detector or gravity-aware trajectory model only when the experiment requires it.
- Keep communication and execution details inside an adapter. The serial codec is an optional interface example.
- Use ROS 2 for target generation and system coordination.
- Use identical units, joint names, target validity rules, and panel geometry in simulation and hardware.

## 5. Essential measurements

| Measurement | Purpose |
| --- | --- |
| Slide position and panel angle against requested targets | Check coordinate signs, tracking error, and feedback source |
| Motion and settling time | Set achievable target deadlines |
| Command-send to backend-acceptance delay | Determine a transport margin for expiring commands |
| Observation timestamp to command acceptance | Identify vision and scheduling latency |
| Time to reach the selected blocking pose | Check whether prediction is early enough |
| Saves divided by attempted valid shots | Evaluate the complete system under stated conditions |

Keep command acceptance, arrival, ball detection, and a successful save as separate outcomes. Label simulated, estimated, and measured axis feedback separately.
