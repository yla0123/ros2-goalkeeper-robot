# MIE1075 Project: Autonomous Goalkeeper Robot

This project is built for the MIE1075 course and aims to develop a simulation of an autonomous robotic goalkeeper for penalty kicks.
## Package Overview

- **`goalkeeper_description`**  
  Contains the physical description of the goalkeeper robot, including the URDF model, mesh files, RGB-D camera, and joint definitions for lateral sliding and body tilting.

- **`goalkeeper_sim`**  
  Contains the Gazebo world, simulation models, ROS-Gazebo bridges, and the main launch file used to start the complete simulation.

- **`penalty_shooter`**  
  Contains the ROS 2 node responsible for generating penalty shots with randomized target positions, flight times, launch velocities, and curved trajectories.

- **`goalkeeper_control`**  
  Contains the low-level goalkeeper control node. It receives desired goalkeeper motion targets and publishes joint trajectory commands for the sliding and tilting joints.

- **`goalkeeper_planner`**  
  Contains the perception and planning components of the goalkeeper system. The current camera viewer node subscribes to the goalkeeper RGB camera stream and displays the incoming images. Future nodes will handle ball detection, 3D position estimation, trajectory prediction, and interception planning.

## System Components

- Soccer field and goal simulation
- Automated randomized penalty shot generation
- RGB-D camera perception
- Camera image visualization
- Goalkeeper lateral and tilting motion control
- Ball detection and tracking *(in development)*
- 3D ball position estimation *(in development)*
- Ball trajectory prediction *(planned)*
- Interception planning *(planned)*

## Build and Launch

### 1. Open the workspace

```bash
cd ~/mie1075_ws
```

### 2. Source ROS 2

```bash
source /opt/ros/jazzy/setup.bash
```

### 3. Build the workspace

```bash
colcon build --symlink-install
```

### 4. Source the workspace

After the build completes:

```bash
source install/setup.bash
```

### 5. Launch the simulation

```bash
ros2 launch goalkeeper_sim goalkeeper.launch.py
```

The launch file starts the main components of the simulation, including:

- Gazebo Harmonic
- Soccer field and goal environment
- Goalkeeper robot
- ROS-Gazebo bridges
- Penalty shooter node

The penalty shooter automatically generates repeated penalty shots during the simulation.

### Rebuilding After Changes

If ROS package configuration, URDF files, launch files, or CMake files are modified, rebuild the workspace:

```bash
cd ~/mie1075_ws
colcon build --symlink-install
source install/setup.bash
```

For Python source changes inside packages built with `--symlink-install`, rebuilding may not always be necessary, but the running node should be restarted before testing the changes.
