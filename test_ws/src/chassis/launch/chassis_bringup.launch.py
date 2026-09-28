from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    chassis_device = LaunchConfiguration("chassis_device")
    chassis_baud_rate = LaunchConfiguration("chassis_baud_rate")
    s21c_port = LaunchConfiguration("s21c_port")
    s21c_module = LaunchConfiguration("s21c_module")
    imu_topic = LaunchConfiguration("imu_topic")

    return LaunchDescription([
        DeclareLaunchArgument(
            "chassis_device",
            default_value=(
                "/dev/serial/by-id/"
                "usb-1a86_USB_Single_Serial_5A6D002531-if00"
            ),
            description="Serial device used by the chassis controller",
        ),
        DeclareLaunchArgument(
            "chassis_baud_rate",
            default_value="115200",
            description="Chassis controller baud rate",
        ),
        DeclareLaunchArgument(
            "s21c_port",
            default_value=(
                "/dev/serial/by-id/"
                "usb-1a86_USB_Single_Serial_597B022936-if00"
            ),
            description="Serial device used by the S21C sensor module",
        ),
        DeclareLaunchArgument(
            "s21c_module",
            default_value="1",
            description="S21C module: 0=ultrasonic, 1=STP23, 2=LD14P",
        ),
        DeclareLaunchArgument(
            "imu_topic",
            default_value="/car/imu_raw",
            description="Raw six-axis Int16MultiArray topic used for heading hold",
        ),
        Node(
            package="chassis",
            executable="chassis_communication",
            name="chassis_communication",
            output="screen",
            parameters=[{
                "device": chassis_device,
                "baud_rate": ParameterValue(chassis_baud_rate, value_type=int),
            }],
        ),
        Node(
            package="chassis",
            executable="wheel_odom",
            name="wheel_odom",
            output="screen",
        ),
        Node(
            package="s21c_receive_data",
            executable="s21c_listener",
            name="s21c_listener",
            output="screen",
            parameters=[{
                "port": s21c_port,
                "module_n": ParameterValue(s21c_module, value_type=int),
            }],
        ),
        Node(
            package="chassis",
            executable="heading_controller",
            name="heading_controller",
            output="screen",
            parameters=[{
                "imu_topic": imu_topic,
                "angular_z_topic": "/heading_controller/angular_z",
                "gyro_full_scale_dps": 500.0,
                "calibration_samples": 200,
                "kp": 1.5,
                "ki": 0.0,
                "kd": 0.1,
                "max_angular_velocity": 1.0,
                "integral_limit": 0.5,
                "yaw_tolerance": 0.01,
            }],
        ),
        Node(
            package="chassis",
            executable="cmd_vel_aggregator",
            name="cmd_vel_aggregator",
            output="screen",
            parameters=[{
                "base_command_topic": "/cmd_vel_input",
                "angular_z_topic": "/heading_controller/angular_z",
                "output_topic": "/cmd_vel",
                "publish_rate": 50.0,
                "base_command_timeout": 0.5,
                "angular_z_timeout": 0.5,
            }],
        ),
    ])
