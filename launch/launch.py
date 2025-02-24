import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('pid_control'),
        'config',
        'parameters.yaml'
    )

    declare_kp = DeclareLaunchArgument('kp', default_value='0.0001', description='kp')
    declare_ki = DeclareLaunchArgument('ki', default_value='0.0005', description='ki')
    declare_kd = DeclareLaunchArgument('kd', default_value='0.0001', description='kd')
    declare_reference = DeclareLaunchArgument('reference', default_value='0', description='reference')

    pid_parameters_override = {
        'pid_control/pid_control/kp': LaunchConfiguration('kp'),
        'pid_control/pid_control/ki': LaunchConfiguration('ki'),
        'pid_control/pid_control/kd': LaunchConfiguration('kd'),
        'pid_control/pid_control/reference': LaunchConfiguration('reference'),
    }

    pid_controller_node = Node(
        package='pid_control',
        executable='pid_controller_node',
        namespace='pid_control',
        name='pid_control',
        parameters=[config, pid_parameters_override]
    )

    return LaunchDescription([
        declare_kp,
        declare_ki,
        declare_kd,
        declare_reference,
        pid_controller_node
    ])