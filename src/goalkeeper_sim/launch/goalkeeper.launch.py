import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Locate the installed simulation and robot-description resources.
    sim_pkg = get_package_share_directory('goalkeeper_sim')
    desc_pkg = get_package_share_directory('goalkeeper_description')

    # Model geometry is generated from config/keeper.design.json.
    world_file = os.path.join(sim_pkg, 'worlds', 'goalkeeper_world.sdf')
    urdf_file = os.path.join(desc_pkg, 'urdf', 'goalkeeper.urdf')
    models_path = os.path.join(sim_pkg, 'models')

    # Add local model resources while preserving existing resource paths.
    set_env_vars = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.pathsep.join(filter(None, [models_path, os.environ.get('GZ_SIM_RESOURCE_PATH', '')]))
    )

    # Publish the generated robot description for spawning and visualization.
    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()

    # This publishes transforms; it does not execute a planned motion.
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content, 'use_sim_time': True}]
    )

    # Start Gazebo Harmonic with the regulation-size goal.
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r "{world_file}"'}.items()
    )

    # Spawn at ground height: the URDF defines its own pivot elevation.
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'goalkeeper_robot',
            '-x', '10.0', '-y', '0.0', '-z', '0.0'
        ],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/goalkeeper_world/wrench@ros_gz_interfaces/msg/EntityWrench]gz.msgs.EntityWrench',
            '/world/goalkeeper_world/remove@ros_gz_interfaces/srv/DeleteEntity',
            '/world/goalkeeper_world/create@ros_gz_interfaces/srv/SpawnEntity',
            '/world/goalkeeper_world/wrench/persistent@ros_gz_interfaces/msg/EntityWrench]gz.msgs.EntityWrench',
            '/world/goalkeeper_world/wrench/clear@ros_gz_interfaces/msg/Entity]gz.msgs.Entity',
        ],
        output='screen'
    )

    penalty_shooter = Node(
        package='penalty_shooter',
        executable='shooter_node',
        output='screen'
    )
    return LaunchDescription([
        set_env_vars,
        gz_sim_launch,
        robot_state_publisher_node,
        spawn_robot,
        bridge,
        penalty_shooter,
    ])
