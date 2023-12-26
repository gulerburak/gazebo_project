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
Connect `base_link` and `chassis_link`.

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


---
Create the launch files directory.
```bash
mkdir -p gazebo_project/launch
```
Create a launch file to launch rviz2
```bash
touch gazebo_project/launch/rviz.launch.py
```
Paste the launch file contents.
**** rviz launch file contents

Add these lines to setup.py to build urdf and launch directories.
```bash
# at the top of setup py, add:
import os
from glob import glob

# in the data_files array, add:
(os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
(os.path.join('share', package_name, 'launch'), glob('launch/*')),
```
Go to your ros2_ws, build the package and source install/setup.bash.
```bash
cd ~/ros2_ws
colcon build --packages-select gazebo_project && source install/setup.bash
```
Run the launch file.
```bash
ros2 launch gazebo_project rviz.launch.py
```
Add set fixed frame to `link_chassis` and add RobotModel visualization. 
