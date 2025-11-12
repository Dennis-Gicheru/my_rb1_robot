import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

<<<<<<< HEAD
def generate_launch_description():
    # Get package description and directory
    package_description = "my_rb1_description"
    package_directory = get_package_share_directory(package_description)

    # Load URDF file
=======
# ROS2 Launch System will look for this function definition #
def generate_launch_description():

    # Get Package Description and Directory #
    package_description = "my_rb1_description"
    package_directory = get_package_share_directory(package_description)

    # Load URDF File #
>>>>>>> spawnrb2
    urdf_file = 'my_rb1_robot.urdf'
    robot_desc_path = os.path.join(package_directory, "urdf", urdf_file)
    print("URDF Loaded !")

<<<<<<< HEAD
    # Joint State Publisher GUI
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # Robot State Publisher
=======
    #Joint State Publisher GUI #
    joint_state_publisher_gui =Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        )

    # Robot State Publisher (RSP) #
>>>>>>> spawnrb2
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        output="screen",
        emulate_tty=True,
<<<<<<< HEAD
        parameters=[{
            'use_sim_time': True,
            'robot_description': Command(['xacro ', robot_desc_path])
        }]
    )

    # RViz2
=======
        parameters=[{'use_sim_time': True, 
                     'robot_description': Command(['xacro ', robot_desc_path])}]
    )

    # RViz2 Launch Configuration (RViz) #
>>>>>>> spawnrb2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_node',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': True}],
    )

<<<<<<< HEAD
    # Return Launch Description
    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui,
        rviz_node
    ])
=======
    # Create and Return the Launch Description Object #
    return LaunchDescription(
        [
            robot_state_publisher_node,
            joint_state_publisher_gui,
            rviz_node
        ]
    )
>>>>>>> spawnrb2
