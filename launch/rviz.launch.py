import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = True
    package_directory = get_package_share_directory("gazebo_project")
    # path of the robot urdf file
    robot_desc_path = os.path.join(package_directory, "urdf", "robot.urdf")
    rviz_config_path = os.path.join(package_directory, "rviz", "config.rviz")

    # Run robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "robot_description": Command(["xacro ", robot_desc_path]),
            }
        ],
    )

    # Run joint state publisher
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher_node",
        output="screen",
        emulate_tty=True,
    )

    # Run Rviz2
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_node",
        output="screen",
        emulate_tty=True,
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=["-d", rviz_config_path],
    )

    return LaunchDescription(
        [
            joint_state_publisher_node,
            robot_state_publisher_node,
            rviz_node,
        ]
    )
