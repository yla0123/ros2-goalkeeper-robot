# Active Simulation Interfaces

Distances are in metres and angles are in radians. ROS timestamps use Gazebo simulation time. The diamond and rectangle are one rigid board.

| Interface | Payload / meaning |
| --- | --- |
| `goal_frame` | Origin at goal-line centre on the ground (world X = 10 m), +X along the shot, +Y lateral, +Z up |
| `slide_joint` | Chassis translation along goal-frame +Y, ±3.55 m, 0.52 m ahead of the goal line |
| `tilt_joint` | Board rotation about +X through the hinge, −90° to +90°. Positive rotation moves the top toward −Y. |
| `/joint_states` | Gazebo's measured `sensor_msgs/msg/JointState` |
| `/keeper/ready` | True when the measured base is centred and nearly still, with no active shot |
| `/keeper/manual_pose` | JSON in `std_msgs/msg/String` with `slide_m` and optional `tilt_rad` defaulting to 0 |
| `/keeper/prediction` | JSON containing `frame_id: "goal_frame"`, `y_m`, `z_m`, and absolute simulation `arrival_time_s` |
| `/keeper/shot` | Internal scripted target with kick stamp and arrival stamp |
| `/model/goalkeeper_robot/joint_trajectory` | Synchronized slide and tilt trajectory, bridged ROS→Gazebo |
| `/model/soccer_ball/pose` | Observed Gazebo ball position |
| `/keeper/shot_result` | Planner reason, command, measured joint feedback and observed goal outcome |

The offline CLI uses kick-relative `--arrival-time-s` and `--elapsed-s`. An optional JSON `robot_state` accepts `slide_m` and `tilt_rad`, which defaults to 0. The adapter rejects stale or moving joint feedback, overlapping commands, unsafe paths and expired deadlines.
