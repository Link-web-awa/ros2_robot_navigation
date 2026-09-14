import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 获取包的路径
    package_dir = get_package_share_directory('s21c_receive_data')

    # 定义launch参数
    module_n_arg = DeclareLaunchArgument(
        'module_n',
        default_value='1',
        description='Module number parameter'
    )

    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyACM0',
        description='Serial device path'
    )

    # 定义节点
    s21c_listener_node = Node(
        package='s21c_receive_data',
        executable='s21c_listener',
        name='s21c_listener',
        output='screen',
        parameters=[{
            'module_n': LaunchConfiguration('module_n'),
            'port': LaunchConfiguration('port')
        }]
    )

    # 创建launch描述
    return LaunchDescription([
        module_n_arg,
        port_arg,
        s21c_listener_node
    ])
