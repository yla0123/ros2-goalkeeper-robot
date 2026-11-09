"""Normal ROS 2 entry point for the generated keeper, controller and optional test shot."""
import json
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

WORLD = "goalkeeper_world"
MODEL = "goalkeeper_robot"


def generate_launch_description():
    sim = Path(get_package_share_directory("goalkeeper_sim"))
    desc = Path(get_package_share_directory("goalkeeper_description"))
    world = sim / "worlds" / "goalkeeper_world.sdf"
    urdf = desc / "urdf" / "goalkeeper.urdf"
    design = json.loads((sim / "config/keeper.design.json").read_text(encoding="utf-8"))
    # The base is centred at the line; the diamond face stays on the pitch side.
    spawn_x = design["geometry"]["goal_plane_world_x_m"]
    gz_launch = Path(get_package_share_directory("ros_gz_sim")) / "launch/gz_sim.launch.py"
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gz_launch)),
        launch_arguments={"gz_args": [LaunchConfiguration("gz_options"), f' "{world}"']}.items())
    base = f"/world/{WORLD}"
    states = f"{base}/model/{MODEL}/joint_state"
    bridge = Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen", arguments=[
        f"{base}/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        f"{states}@sensor_msgs/msg/JointState[gz.msgs.Model",
        f"/model/{MODEL}/joint_trajectory@trajectory_msgs/msg/JointTrajectory]gz.msgs.JointTrajectory",
        "/model/soccer_ball/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose",
        f"{base}/wrench@ros_gz_interfaces/msg/EntityWrench]gz.msgs.EntityWrench",
        f"{base}/remove@ros_gz_interfaces/srv/DeleteEntity",
        f"{base}/create@ros_gz_interfaces/srv/SpawnEntity",
    ], remappings=[(states, "/joint_states"), (f"{base}/clock", "/clock")])
    return LaunchDescription([
        DeclareLaunchArgument("gz_options", default_value="-r"),
        DeclareLaunchArgument("shoot", default_value="false"),
        DeclareLaunchArgument("shot_case", default_value="center"),
        SetEnvironmentVariable(name="GZ_SIM_RESOURCE_PATH",
                               value=os.pathsep.join(filter(None, [str(sim / "models"), str(desc.parent),
                                                                   os.environ.get("GZ_SIM_RESOURCE_PATH")]))),
        gz,
        Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen",
             parameters=[{"robot_description": urdf.read_text(encoding="utf-8"), "use_sim_time": True}]),
        Node(package="ros_gz_sim", executable="create", output="screen",
             arguments=["-world", WORLD, "-file", str(urdf), "-name", MODEL, "-x", str(spawn_x),
                        "-y", "0", "-z", "0"]),
        bridge,
        Node(package="goalkeeper_sim", executable="keeper_sim_node", output="screen",
             parameters=[{"use_sim_time": True}]),
        Node(package="penalty_shooter", executable="shooter_node", output="screen",
             condition=IfCondition(LaunchConfiguration("shoot")),
             parameters=[{"use_sim_time": True, "single_shot": True, "require_keeper_ready": True,
                          "shot_case": LaunchConfiguration("shot_case")}]),
    ])
