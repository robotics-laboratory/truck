from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
import xacro

NAMESPACE = 'truck'
OUTPUT = 'screen'


def generate_launch_description():
    gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource([
        os.path.join(get_package_share_directory('gazebo_ros'), 'launch'),
        '/gazebo.launch.py',
    ]))

    truck_description_path = os.path.join(get_package_share_directory('truck_description'))

    xacro_file = os.path.join(truck_description_path, 'urdf', 'truck.xacro')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': doc.toxml()}],
        output=OUTPUT,
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', f'robot_description', '-entity', 'truck'],
        output=OUTPUT,
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'joint_state_broadcaster'],
        output=OUTPUT,
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'joint_trajectory_controller'],
        output=OUTPUT,
    )

    load_velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'velocity_controller'],
        output=OUTPUT,
    )

    control_node = Node(
        package='control_node',
        executable='control_node',
        output=OUTPUT,
        namespace=NAMESPACE,
    )

    gzweb = ExecuteProcess(
        cwd="/opt/gzweb",
        cmd=['npm', 'start'],
        output='log',
        shell=True,
    )

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_joint_trajectory_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_trajectory_controller,
                on_exit=[load_velocity_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_velocity_controller,
                on_exit=[control_node],
            )
        ),
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        gzweb,
    ])