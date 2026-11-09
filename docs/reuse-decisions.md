# Reuse Decisions

Repository inspection date: October 10, 2026. These decisions are based on source inspection and author documentation, not a local hardware demonstration.

## 1. Selected building blocks

| Component | Choice | Reason | Work still required |
| --- | --- | --- | --- |
| Gazebo integration | [ros_gz_bridge for Jazzy](https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_bridge/README.md) | Bridges joint commands, state, and ball pose | Validate startup and feedback on Ubuntu |
| Simulated joint motion | [Gazebo Harmonic joint trajectory controller](https://gazebosim.org/api/sim/8/jointcontrollers.html) | Executes a lateral trajectory on the fixed-panel keeper | Measure and tune tracking in the simulator |

Pin tested simulation-library versions. Existing library examples show individual capabilities; they do not prove the complete interception pipeline meets its timing budget.

## 2. Reference goalkeeper projects

| Repository | Useful part | Verified limitation |
| --- | --- | --- |
| [CSE-ICE-22/GoalKeeper](https://github.com/CSE-ICE-22/GoalKeeper) | ROS 2 camera bring-up and red-ball 3D detection | Public tree lacks the described EKF and complete motion-control implementation |
| [nanmu42/robo-playground](https://github.com/nanmu42/robo-playground) | Feedback-aware keeper control and recentering | Bound to a DJI RoboMaster EP wheeled platform; different mechanics |
| [Gautham-S0117/Orien-Robotic-Goalkeeper](https://github.com/Gautham-S0117/Orien-Robotic-Goalkeeper) | HSV vision and goal-line prediction reference | Controller implementation is incomplete; Python sends left/right/stop rather than absolute positions |
| [oguzhan-gun/robokeeper-stereo-vision](https://github.com/oguzhan-gun/robokeeper-stereo-vision) | Demonstration of stereo prediction concepts | Repository explicitly omits implementation source code |

Use these projects as references for perception, prediction, and software interfaces. Before copying third-party implementation, check the applicable license. The CSE detector package declares Apache-2.0 locally; that declaration should not be assumed to cover every repository component.

## 3. Commercial reference

The [RobotKeeper product page](https://www.robokeeper.cn/cn/products/football/) describes vision, interception prediction, and rotating or sliding blocking mechanisms. It does not establish that each standard product combines both independent axes. Its industrial actuator scale and advertised performance are design context, not measurements for this prototype.

## 4. Project-specific work

The team still needs to validate tracking, timing and experimental outcomes, then connect calibrated camera prediction. The scripted shooter is a source of test targets, not a substitute for camera validation.
