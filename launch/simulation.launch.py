import os
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    package_description = "gazebo_project"
    package_directory = get_package_share_directory(package_description)

    # Set the Path to Robot Mesh Models for Loading in Gazebo Sim
    install_dir_path = get_package_prefix(package_description) + "/share"
    robot_meshes_path = os.path.join(package_directory, "meshes")
    gazebo_resource_paths = [install_dir_path, robot_meshes_path]
    if "IGN_GAZEBO_RESOURCE_PATH" in os.environ:
        for resource_path in gazebo_resource_paths:
            if resource_path not in os.environ["IGN_GAZEBO_RESOURCE_PATH"]:
                os.environ["IGN_GAZEBO_RESOURCE_PATH"] += ":" + resource_path
    else:
        os.environ["IGN_GAZEBO_RESOURCE_PATH"] = ":".join(gazebo_resource_paths)

    # Load Demo World SDF from Robot Description Package
    world_file = "demo_world.sdf"
    world_file_path = os.path.join(package_directory, "worlds", world_file)
    world_config = LaunchConfiguration("world")
    declare_world_arg = DeclareLaunchArgument(
        "world", default_value=["-r ", world_file_path], description="SDF World File"
    )

    # Declare Gazebo Sim Launch file
    gzsim_pkg = get_package_share_directory("ros_gz_sim")
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([gzsim_pkg, "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={"gz_args": world_config}.items(),
    )

    # Load the urdf
    urdf_file = "robot.urdf"
    robot_desc_path = os.path.join(package_directory, "urdf", urdf_file)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "use_sim_time": True,
                "robot_description": Command(["xacro ", robot_desc_path]),
            }
        ],
    )

    # declare robot spawn position
    declare_spawn_x = DeclareLaunchArgument(
        "x", default_value="0.0", description="Model Spawn X Axis Value"
    )
    declare_spawn_y = DeclareLaunchArgument(
        "y", default_value="0.0", description="Model Spawn Y Axis Value"
    )
    declare_spawn_z = DeclareLaunchArgument(
        "z", default_value="0.5", description="Model Spawn Z Axis Value"
    )
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        name="my_robot_spawn",
        arguments=[
            "-name",
            "my_robot",
            "-allow_renaming",
            "true",
            "-topic",
            "robot_description",
            "-x",
            LaunchConfiguration("x"),
            "-y",
            LaunchConfiguration("y"),
            "-z",
            LaunchConfiguration("z"),
        ],
        output="screen",
    )

    ign_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ign_bridge",
        arguments=[
            "/clock" + "@rosgraph_msgs/msg/Clock" + "[ignition.msgs.Clock",
            "/cmd_vel" + "@geometry_msgs/msg/Twist" + "@ignition.msgs.Twist",
            "/tf" + "@tf2_msgs/msg/TFMessage" + "[ignition.msgs.Pose_V",
            "/odom" + "@nav_msgs/msg/Odometry" + "[ignition.msgs.Odometry",
            "/world/empty/model/my_robot/joint_state"
            + "@sensor_msgs/msg/JointState"
            + "[ignition.msgs.Model",
            "/imu" + "@sensor_msgs/msg/Imu" + "[ignition.msgs.IMU",
            "/laser/scan" + "@sensor_msgs/msg/LaserScan" + "[ignition.msgs.LaserScan",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            SetParameter(name="use_sim_time", value=True),
            declare_world_arg,
            gz_sim,
            robot_state_publisher_node,
            declare_spawn_x,
            declare_spawn_y,
            declare_spawn_z,
            gz_spawn_entity,
            ign_bridge,
        ]
    )
