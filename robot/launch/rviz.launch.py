import os

from ament_index_python.packages import get_package_share_directory

from launch.substitutions import Command
import launch
import launch_ros.actions


def generate_launch_description():
    share_dir = get_package_share_directory('robot')

    xacro_file = os.path.join(share_dir, 'robot.urdf.xacro')

    rsp_params = {'robot_description': Command(['xacro',' ',xacro_file])}
    rsp = launch_ros.actions.Node(package='robot_state_publisher',
    executable='robot_state_publisher',
    output='both',
    parameters=[rsp_params])

    jsp_gui = launch_ros.actions.Node(
        package='joint_state_publisher.gui',
        executable='joint_state_publisher.gui',
        name='joint_state_publisher.gui'
    )

    sp = launch_ros.actions.Node(package='robot',
    executable='state_publisher',
    output='both')

    rviz_conf_file = os.path.join(share_dir, 'robot.rviz')
    rviz = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_conf_file],
        output='screen')

    
    return launch.LaunchDescription([rsp, sp, rviz])
