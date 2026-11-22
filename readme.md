# MIE1075 Project: Autonomous Goalkeeper Robot

This project is built for the MIE1075 course and aims to develop a simulation of an autonomous robotic goalkeeper for penalty kicks.

## Environment

The project is developed and tested using the following environment:

- **Operating System:** Ubuntu 24.04 LTS (WSL2)
- **ROS 2:** Jazzy Jalisco
- **Simulator:** Gazebo Harmonic
- **Build System:** `colcon`
- **Languages:** C++ and Python

## Package Overview

- **`goalkeeper_description`**  
  Contains the physical description of the goalkeeper robot, including its URDF model and mesh files.

- **`goalkeeper_sim`**  
  Contains the Gazebo world, simulation models, and the main launch file used to start the complete simulation.

- **`penalty_shooter`**  
  Contains the ROS 2 node responsible for generating penalty shots with randomized target positions, flight times, and curved trajectories.

## Planned Components

- Soccer field and goal simulation
- Penalty shot generation
- Ball detection and tracking
- Ball trajectory prediction
- Goalkeeper localization
- Interception planning
- Goalkeeper motion control

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
