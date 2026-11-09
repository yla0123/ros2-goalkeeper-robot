# Active Simulation Interfaces

Distances are in metres. ROS timestamps use Gazebo simulation time. The fixed diamond and lower rectangle do not have separate commands.

| Interface | Payload / meaning |
| --- | --- |
| `goal_frame` | Origin at goal-line centre on the ground (world X = 10 m), +X along the shot, +Y lateral, +Z up |
| `slide_joint` | Sole moving joint, lateral ±2.65 m |
| `/joint_states` | Gazebo's measured `sensor_msgs/msg/JointState` |
| `/keeper/ready` | True when the measured base is centred and nearly still, with no active shot |
| `/keeper/manual_pose` | JSON in `std_msgs/msg/String` with one field, `slide_m` |
| `/keeper/prediction` | JSON containing `frame_id: "goal_frame"`, `y_m`, `z_m`, and absolute simulation `arrival_time_s` |
| `/keeper/shot` | Internal scripted target with kick stamp and arrival stamp |
| `/model/goalkeeper_robot/joint_trajectory` | Checked one-joint trajectory, bridged ROS→Gazebo |
| `/model/soccer_ball/pose` | Observed Gazebo ball position |
| `/keeper/shot_result` | Planner reason, command, measured joint feedback and observed goal outcome |

The offline CLI uses kick-relative `--arrival-time-s` and `--elapsed-s`. An optional JSON `robot_state` contains only `slide_m`. The adapter does not send a trajectory for stale or moving joint feedback, overlapping commands, unsafe paths or expired deadlines. Camera prediction and hardware actuation are outside this update.
