# Multi Arm Teleoperation with Hand-Tracking via MediaPipe

The basic Idea of this project is to do the `teleopration` in cartesian space by moving the end effector  
in a desired position in x and Y. Since the `Depth Camera` is not available, Z is not used; limiting DOF of the  
robots to 3.

## Setup

The setup includes two 6 DOF `Franka_eamika research 3` robots placed at 0.75m distance facing each other in x axis.  
The 7th Revolute joint in `Franka` is set to fixed in URDF to mimic `6 DOF`. 

## Requirements

ROS2 Setup:

To run the setup, You need to have ROS2 Humble setup with Moveit2 motion planning working.  
Make sure to download all the controllers and additional packages that are listed in package.xml. 

The important one is `Joint Trajectory Controller`

```bash
sudo apt update && sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers 
```
--------------------------------------------------------------------------------------------------------
Camera Setup:

Camera setup is in steps to teleoperate

----------------------------------------------------------------------------------------------------------
Hand Tracking : 
From pip , please install mediapipe by executing:

```bash
pip install --upgrade pip
pip install mediapipe opencv-python
```


## Steps to Teleoperate 

1) Clone the repo and build ros2 workspace

```bash
git clone https://github.com/dhruvil122/hand_teleop.git
cd hand_teleop
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```
2) setup the camera feedback

If you do not have direct camera access, Please use 
```bash 
python3 src/robot_bringup/scripts/flask_bringup.py 
```

This script will start publishing the Image to your IP. retrive the IP from the `ifconfig`.

It should look something like this -> "192.168.64.1". The port is 5000, hence the full address should be "http://$IPAddress:5000/video_feed"
Please change it in the main hand_tracking node, in Line 27.


3) launch Movegroup, Rviz, Hand Tracking and Teleoperation Node togather

```bash
ros2 launch moveit_panda moveit_rviz.launch.py arm_id:=multi_arm load_gripper:=true
```

4) Wait for 10seconds(moveit_servo wait time. Can be changed in the launch file) for Teleoperation service to start and then start moving your hands. You can move both of your hands at the same time. Both robots can work togather.








