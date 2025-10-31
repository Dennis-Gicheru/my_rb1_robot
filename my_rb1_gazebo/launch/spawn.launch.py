import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Locate the description package
    package_description = "my_rb1_description"
    package_directory = get_package_share_directory(package_description)

    # Path to URDF
    urdf_file = 'my_rb1_robot.urdf'
    robot_desc_path = os.path.join(package_directory, "urdf", urdf_file)

    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': True,
            'robot_description': ParameterValue(
                Command(['cat ', robot_desc_path]),
                value_type=str
            )
        }]
    )

    # Declare spawn position arguments
    declare_spawn_x = DeclareLaunchArgument("x", default_value="0.0")
    declare_spawn_y = DeclareLaunchArgument("y", default_value="0.0")
    declare_spawn_z = DeclareLaunchArgument("z", default_value="0.5")

    # Spawn entity in Gazebo
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        name="my_robot_spawn",
        arguments=[
            "-name", "my_robot",
            "-allow_renaming", "true",
            "-topic", "robot_description",
            "-x", LaunchConfiguration("x"),
            "-y", LaunchConfiguration("y"),
            "-z", LaunchConfiguration("z"),
        ],
        output="screen",
    )

    # ROS–Gazebo bridge
    ign_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ign_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock]",
            "/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist",
            "/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V]",
            "/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry]",
            "/laser/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan]",
            "/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU]",
        ],
        output="screen",
    )

    # ✅ Return LaunchDescription (must be the last statement)
    return LaunchDescription([
        SetParameter(name="use_sim_time", value=True),
        declare_spawn_x,
        declare_spawn_y,
        declare_spawn_z,
        robot_state_publisher_node,
        gz_spawn_entity,
        ign_bridge,
    ])
