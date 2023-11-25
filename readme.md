# Gazebo Tutorial Course 
## Creating the ros package
Go to the your ros2 workspace and create a Python package. `urdf` and `xacro` packages are dependencies of this package.
```bash
cd ~/ros2_ws/src
ros2 pkg create gazebo_project --build-type ament_python --dependencies rclpy urdf xacro
```
