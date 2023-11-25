# Gazebo Tutorial Course 
## Dependencies
You need to install ROS 2 humble for this project.
## Creating the ros package
Go to the your ros2 workspace and create a Python package. `urdf` and `xacro` packages are dependencies of this package.
```bash
cd ~/ros2_ws/src \
ros2 pkg create gazebo_project --build-type ament_python --dependencies rclpy urdf xacro
```
Create the URDF directory
```bash
mkdir -p gazebo_project/urdf
```

Create the robot urdf file
```bash
touch gazebo_project/urdf/robot.urdf
```
******Fill the `robot.urdf` file.

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
