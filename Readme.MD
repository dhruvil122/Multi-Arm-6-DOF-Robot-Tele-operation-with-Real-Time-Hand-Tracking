# Multi Arm Teleoperation with Hand-Tracking via MediaPipe

The basic Idea of this project is to do the `teleopration` in cartesian space by moving the end effector  
in a desired position in x and Y. Since the `Depth Camera` is not available, Z is not used; limiting DOF of the  
robots to 3.

## Setup

The setup includes two 6 DOF `Franka_eamika research 3` robots placed at 0.75m distance facing each other in x axis.  
The 7th Revolute joint in `Franka` is set to fixed in URDF to mimic `6 DOF`. 

## Requirements

To run the setup, You need to have ROS2 Humble setup with Moveit2 motion planning working.  
Make sure to download all the controllers and additional packages that are listed in package.xml.

The important one is `Joint Trajectory Controller`

```bash
sudo apt update && sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers 
```


From pip package, please install mediapipe by executing:

```bash
pip install --upgrade pip
pip install mediapipe opencv-python
```
