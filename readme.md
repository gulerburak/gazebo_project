# Gazebo Tutorial Course 

## Dependencies

You need to install ROS 2 humble and Gazebo Sim for this project.
    - [ROS2 Humble installation instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
    - [Gazebo Sim installation instructions](https://gazebosim.org/docs/harmonic/install_ubuntu)
## Creating the ros package
Go to the your ros2 workspace and create a Python package. `urdf` and `xacro` packages are dependencies of this package.
```bash
cd ~/ros2_ws/src \
ros2 pkg create gazebo_project --build-type ament_python --dependencies rclpy urdf xacro
```
Create the URDF directory to put the robot's URDF file
```bash
mkdir -p gazebo_project/urdf
```

Create the robot's urdf file
```bash
touch gazebo_project/urdf/robot.urdf
```
Create the world directory to put the simulation world file
```bash
mkdir -p gazebo_project/worlds
```

Create the `demo_world.sdf` file
```bash
touch gazebo_project/worlds/demo_world.sdf
```

Create the launch directory to put the launch files
```bash
mkdir -p gazebo_project/launch
```

Create the `rviz.launch.py` file to inspect robot's urdf with rviz.
```bash
touch gazebo_project/launch/rviz.launch.py
```
Create the `simulation.launch.py` to run simulation with created map and robot.
```bash
touch gazebo_project/launch/simulation.launch.py
```
Create the meshes directory to put the robot's .obj file
```bash
mkdir -p gazebo_project/meshes
```

Then, move `smart_car.obj` file from this repo to `gazebo_project/meshes` directory.

---
## Creating the robot
Inside of the URDF file, add the xml and robot tags.

```xml
<?xml version="1.0"?>
<robot name="robot">

</robot>
```

**The rest of pieces will be added inside the `<robot></robot>` tag.**
Adding the dummy base link:
```xml
    <link name="base_link"></link>
```
Adding the chassis link.
Chassis' visual will be a .obj file named smart_car.obj that located in meshes folder in the project. The material tag that located inside of the visual properties ensure the lightning of the car is correct and working. 
```xml
<link name="link_chassis">
    <inertial>
      <mass value="500.0" />
      <origin xyz="0 0 0.46" rpy="0 0 0" />
      <inertia ixx="1000.0" ixy="0.0" iyy="200.0" ixz="0.0" iyz="0.0" izz="1000.0" />
    </inertial>

    <collision>
      <origin xyz="0 0 0.625" rpy="0 0 0" />
      <geometry>
        <box size="1.0 1.0 0.5" />
      </geometry>
    </collision>

    <visual name="chassis_visual">
      <origin rpy="1.57 0 1.57" xyz="0 0 0" />
      <geometry>
        <mesh filename="package://gazebo_project/meshes/smart_car.obj" scale="0.012 0.012 0.013" />
      </geometry>
      <material name="white">
        <lighting>true</lighting>
        <cast_shadows>true</cast_shadows>
        <receive_shadows>true</receive_shadows>
        <color rgba="0.9 0.9 0.9 1.0" />
      </material>
    </visual>
  </link>
```
Connect `base_link` and `link_chassis`.

```xml
  <joint name="joint_base_link_chassis" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0" />
    <child link="link_chassis" />
    <parent link="base_link" />
  </joint>

```

**Adding Wheel Links**

- Rear left wheel link
```xml
  <link name="rear_left_wheel">
    <inertial>
      <mass value="11.0" />
      <inertia ixx="0.58" ixy="0.0" iyy="0.33" ixz="0.0" iyz="0.0" izz="0.33" />
    </inertial>

    <visual name="rear_left_wheel_visual">
      <geometry>
        <cylinder radius="0.25" length="0.15" />
      </geometry>
      <material name="Black">
        <color rgba="0.1 0.1 0.1 1" />
      </material>
    </visual>

    <collision>
      <geometry>
        <sphere radius="0.25" />
      </geometry>
      <friction>
        <ode>
          <mu>1.1</mu>
          <mu2>1.1</mu2>
        </ode>
      </friction>
    </collision>
  </link>
```

- Rear right wheel link
```xml
  <link name="rear_right_wheel">
    <inertial>
      <mass value="11.0" />
      <inertia ixx="0.58" ixy="0.0" iyy="0.33" ixz="0.0" iyz="0.0" izz="0.33" />
    </inertial>

    <visual name="rear_right_wheel_visual">
      <geometry>
        <cylinder radius="0.25" length="0.15" />
      </geometry>
      <material name="Black">
        <color rgba="0.1 0.1 0.1 1" />
      </material>
    </visual>

    <collision>
      <geometry>
        <sphere radius="0.25" />
      </geometry>
      <friction>
        <ode>
          <mu>1.1</mu>
          <mu2>1.1</mu2>
        </ode>
      </friction>
    </collision>
  </link>
```

- Front left wheel link
```xml
  <link name="front_left_wheel">
    <inertial>
      <mass value="11.0" />
      <inertia ixx="0.58" ixy="0.0" iyy="0.33" ixz="0.0" iyz="0.0" izz="0.33" />
    </inertial>

    <visual name="front_left_wheel_visual">
      <geometry>
        <cylinder radius="0.25" length="0.15" />
      </geometry>
      <material name="Black">
        <color rgba="0.1 0.1 0.1 1" />
      </material>
    </visual>

    <collision>
      <geometry>
        <sphere radius="0.25" />
      </geometry>
      <friction>
        <ode>
          <mu>1.1</mu>
          <mu2>1.1</mu2>
        </ode>
      </friction>
    </collision>
  </link>
```

- Front right wheel link
```xml
  <link name="front_right_wheel">
    <inertial>
      <mass value="11.0" />
      <inertia ixx="0.58" ixy="0.0" iyy="0.33" ixz="0.0" iyz="0.0" izz="0.33" />
    </inertial>

    <visual name="front_right_wheel_visual">
      <geometry>
        <cylinder radius="0.25" length="0.15" />
      </geometry>
      <material name="Black">
        <color rgba="0.1 0.1 0.1 1" />
      </material>
    </visual>

    <collision>
      <geometry>
        <sphere radius="0.25" />
      </geometry>
      <friction>
        <ode>
          <mu>1.1</mu>
          <mu2>1.1</mu2>
        </ode>
      </friction>
    </collision>
  </link>
```

The front wheels also need a steering link.

- Front left wheel steering link 
```xml
  <link name="front_left_wheel_steering_link">
    <inertial>
      <mass value="1.0" />
      <inertia ixx="0.58" ixy="0.0" iyy="0.33" ixz="0.0" iyz="0.0" izz="0.33" />
    </inertial>
  </link>
```
- Front right wheel steering link
```xml
  <link name="front_right_wheel_steering_link">
    <inertial>
      <mass value="1.0" />
      <inertia ixx="0.58" ixy="0.0" iyy="0.33" ixz="0.0" iyz="0.0" izz="0.33" />
    </inertial>
  </link>
```

**Connecting Wheel Links to the Chassis**
- Connecting the `rear_left_wheel` and `link_chassis`.
```xml
  <joint type="revolute" name="rear_left_wheel_joint">
    <origin rpy="1.56 0 0" xyz="-0.9 0.64 0.25" />
    <child link="rear_left_wheel" />
    <parent link="link_chassis" />
    <axis rpy="0 0 0" xyz="0 0 1" />
    <limit lower="-1.79769e+308" upper="1.79769e+308"
      effort="100000" velocity="1000" />
  </joint>
```
- Connecting the `rear_right_wheel` and `link_chassis`.
```xml
  <joint type="revolute" name="rear_right_wheel_joint">
    <origin rpy="1.56 0 0" xyz="-0.9 -0.64 0.25" />
    <child link="rear_right_wheel" />
    <parent link="link_chassis" />
    <axis rpy="0 0 0" xyz="0 0 1" />
    <limit lower="-1.79769e+308" upper="1.79769e+308"
      effort="100000" velocity="1000" />
  </joint>
```
- Connecting the `front_left_wheel_steering_link` and `link_chassis`.
```xml
  <joint type="revolute" name="front_left_wheel_steering_joint">
    <origin rpy="-1.56 0 0" xyz="0.9 0.64 0.25" />
    <child link="front_left_wheel_steering_link" />
    <parent link="link_chassis" />
    <axis rpy="0 0 0" xyz="0 -1 0" />
    <limit effort="100000" velocity="1000" upper="0.8727" lower="-0.8727" />
  </joint>
```
- Connecting the `front_left_wheel` and `front_left_wheel_steering_link`.
```xml
  <joint type="revolute" name="front_left_wheel_rotating_joint">
    <origin rpy="0 0 0" xyz="0 0 0" />
    <child link="front_left_wheel" />
    <parent link="front_left_wheel_steering_link" />
    <axis rpy="0 0 0" xyz="0 0 1" />
    <limit lower="-1.79769e+308" upper="1.79769e+308" effort="100000" velocity="1000" />
  </joint>
```
- Connecting the `front_right_wheel_steering_link` and `link_chassis`.
```xml
  <joint type="revolute" name="front_right_wheel_steering_joint">
    <origin rpy="-1.56 0 0" xyz="0.9 -0.64 0.25" />
    <child link="front_right_wheel_steering_link" />
    <parent link="link_chassis" />
    <axis rpy="0 0 0" xyz="0 -1 0" />
    <limit effort="100000" velocity="1000" upper="0.8727" lower="-0.8727" />
  </joint>
```
- Connecting the `front_right_wheel` and `front_right_wheel_steering_link`.
```xml
  <joint type="revolute" name="front_right_wheel_rotating_joint">
    <origin rpy="0 0 0" xyz="0 0 0" />
    <child link="front_right_wheel" />
    <parent link="front_right_wheel_steering_link" />
    <axis rpy="0 0 0" xyz="0 0 1" />
    <limit lower="-1.79769e+308" upper="1.79769e+308" effort="100000" velocity="1000" />
    <joint_properties damping="1.0" friction="1.0" />
  </joint>
```
**Adding the Ackermann Steering System Plugin**

After adding this plugin, car can be drivable on `/cmd_vel` topic with `Twist` messages. Odometry of the vehicle will be published in `/odom` topic. `angular.z` value of `Twist` message will steer `front_left_wheel_steering_joint` and `front_right_wheel_steering_joint`, `linear.x` value of `Twist` mesage will rotate `front_left_wheel_rotating_joint` and `front_right_wheel_rotating_joint`.

```xml
  <gazebo>
    <plugin filename="ignition-gazebo-ackermann-steering-system"
      name="ignition::gazebo::systems::AckermannSteering">
      <topic>/cmd_vel</topic>
      <odom_publish_frequency>10</odom_publish_frequency>
      <odom_topic>odom</odom_topic>
      <left_joint>front_left_wheel_rotating_joint</left_joint>
      <right_joint>front_right_wheel_rotating_joint</right_joint>
      <left_steering_joint>front_left_wheel_steering_joint</left_steering_joint>
      <right_steering_joint>front_right_wheel_steering_joint</right_steering_joint>
      <kingpin_width>1.0</kingpin_width>
      <steering_limit>0.8</steering_limit>
      <wheel_base>1.0</wheel_base>
      <wheel_separation>1.572</wheel_separation>
      <wheel_radius>0.31265</wheel_radius>
      <min_velocity>-10</min_velocity>
      <max_velocity>100</max_velocity>
      <min_acceleration>-30</min_acceleration>
      <max_acceleration>30</max_acceleration>
    </plugin>
  </gazebo>
```
Adding Joint State Publihser plugin to publihs real-time joint states.
```xml
  <gazebo>
    <plugin filename="ignition-gazebo-joint-state-publisher-system"
      name="ignition::gazebo::systems::JointStatePublisher" />
  </gazebo>
```

Adding IMU Sensor plugin

```xml
  <gazebo>
    <plugin filename="ignition-gazebo-imu-system"
      name="ignition::gazebo::systems::Imu">
    </plugin>
  </gazebo>
  <gazebo reference="link_chassis">
    <sensor name="imu_sensor" type="imu">
      <always_on>1</always_on>
      <update_rate>20</update_rate>
      <visualize>true</visualize>
      <topic>imu</topic>
    </sensor>
  </gazebo>
```
Our car is created. To test it, fill the `rviz.launch.py` with following lines:
```python
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
    )

    return LaunchDescription(
        [
            joint_state_publisher_node,
            robot_state_publisher_node,
            rviz_node,
        ]
    )
```

Do the [Building and testing steps](#building)

--- 
# Creating the Simulation World

---
# Building 

To build the project, it is needed to add the new directories to the `setup.py`

Add these lines to setup.py to build urdf, worlds, meshes and launch directories.
```bash
# at the top of setup py, add:
import os
from glob import glob

# in the data_files array, add:
        (os.path.join("share", package_name, "urdf"), glob("urdf/*")),
        (os.path.join("share", package_name, "meshes"), glob("meshes/*")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*")),
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
```

Then, build the package and source install/setup.bash.
```bash
colcon build --packages-select gazebo_project && source install/setup.bash
```
****IMPORTANT****: If you make any changes to the files in this project, you need to do the build and source step!

- To test to URDF file, run:
```bash
ros2 launch gazebo_project rviz.launch.py
```
Add set fixed frame to `link_chassis` and add RobotModel visualization. 

- To test the whole simulation, run:
```bash
ros2 launch gazebo_project simulation.launch.py
```

Do not run these 2 launch files at the same time.



