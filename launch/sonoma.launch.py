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
    world = "sonoma"

    world_file = f"{world}.sdf"
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
        "x", default_value="277.88", description="Model Spawn X Axis Value"
    )
    declare_spawn_y = DeclareLaunchArgument(
        "y", default_value="-135.2", description="Model Spawn Y Axis Value"
    )
    declare_spawn_z = DeclareLaunchArgument(
        "z", default_value="3.0", description="Model Spawn Z Axis Value"
    )
    declare_spawn_R = DeclareLaunchArgument(
        "R", default_value="0.0", description="Model Spawn Roll Value"
    )
    declare_spawn_P = DeclareLaunchArgument(
        "P", default_value="0.02", description="Model Spawn Pitch Value"
    )
    declare_spawn_Y = DeclareLaunchArgument(
        "Y", default_value="-0.66", description="Model Spawn Yaw Value"
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
            "-R",
            LaunchConfiguration("R"),
            "-P",
            LaunchConfiguration("P"),
            "-Y",
            LaunchConfiguration("Y"),
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
            f"/world/{world}/model/my_robot/joint_state"
            + "@sensor_msgs/msg/JointState"
            + "[ignition.msgs.Model",
            "/imu" + "@sensor_msgs/msg/Imu" + "[ignition.msgs.IMU",
            # "/laser/scan" + "@sensor_msgs/msg/LaserScan" + "[ignition.msgs.LaserScan",
            f"/world/{world}/pose/info"
            + "@geometry_msgs/msg/PoseArray"
            + "[ignition.msgs.Pose_V",
        ],
        remappings=[
            (f"/world/{world}/model/my_robot/joint_state", "/joint_states"),
            (f"/world/{world}/pose/info", "/pose_info"),
        ],
        output="screen",
    )

    cam_tf_node = Node(
        name="camera_stf",
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=[
            "0",
            "0",
            "0",
            "1.5707",
            "-1.5707",
            "0",
            "oakd_rgb_camera_optical_frame",
            "/my_robot/oakd_rgb_camera_frame/rgbd_camera",
        ],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
    )

    robot_name = "my_robot"

    oakd_camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="camera_bridge",
        output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=[
            [
                "/world/",
                world,
                "/model/",
                robot_name,
                "/link/oakd_rgb_camera_frame/sensor/rgbd_camera/image"
                + "@sensor_msgs/msg/Image"
                + "[ignition.msgs.Image",
            ],
            [
                "/world/",
                world,
                "/model/",
                robot_name,
                "/link/oakd_rgb_camera_frame/sensor/rgbd_camera/depth_image"
                + "@sensor_msgs/msg/Image"
                + "[ignition.msgs.Image",
            ],
            [
                "/world/",
                world,
                "/model/",
                robot_name,
                "/link/oakd_rgb_camera_frame/sensor/rgbd_camera/points"
                + "@sensor_msgs/msg/PointCloud2"
                + "[ignition.msgs.PointCloudPacked",
            ],
            [
                "/world/",
                world,
                "/model/",
                robot_name,
                "/link/oakd_rgb_camera_frame/sensor/rgbd_camera/camera_info"
                + "@sensor_msgs/msg/CameraInfo"
                + "[ignition.msgs.CameraInfo",
            ],
        ],
        remappings=[
            (
                [
                    "/world/",
                    world,
                    "/model/",
                    robot_name,
                    "/link/oakd_rgb_camera_frame/sensor/rgbd_camera/image",
                ],
                "oakd/rgb/image_raw",
            ),
            (
                [
                    "/world/",
                    world,
                    "/model/",
                    robot_name,
                    "/link/oakd_rgb_camera_frame/sensor/rgbd_camera/depth_image",
                ],
                "oakd/rgb/depth",
            ),
            (
                [
                    "/world/",
                    world,
                    "/model/",
                    robot_name,
                    "/link/oakd_rgb_camera_frame/sensor/rgbd_camera/points",
                ],
                "oakd/rgb/depth/points",
            ),
            (
                [
                    "/world/",
                    world,
                    "/model/",
                    robot_name,
                    "/link/oakd_rgb_camera_frame/sensor/rgbd_camera/camera_info",
                ],
                "oakd/rgb/camera_info",
            ),
        ],
    )

    odometry_tf = Node(
        package="gazebo_project",
        executable="odometry_tf",
        name="odometry_tf",
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
            declare_spawn_R,
            declare_spawn_P,
            declare_spawn_Y,
            gz_spawn_entity,
            ign_bridge,
            cam_tf_node,
            oakd_camera_bridge,
            odometry_tf,
        ]
    )
