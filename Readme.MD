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
    Case 1) Do not have a direct Camera access?

        If you do not have direct camera access, Please use 
        ```bash 
        python3 src/robot_bringup/scripts/flask_bringup.py 
        ```
        
        This script will start publishing the Image to your IP. retrive the IP from the `ifconfig`.
        
        It should look something like this -> "192.168.64.1". The port is 5000, hence the full address should be "http://$IPAddress:5000/video_feed"
        

    Case 2) Have direct access to the camera ? 

        Good. Then you only have to commentout line no. 59 and comment line no. 60. The camera setup should work.


3) launch Movegroup, Rviz, Hand Tracking and Teleoperation Node togather

```bash
ros2 launch moveit_panda moveit_rviz.launch.py arm_id:=multi_arm load_gripper:=true
```

4) Wait for 10seconds(moveit_servo wait time. Can be changed in the launch file) for Teleoperation service to start and then start moving your hands. You can move both of your hands at the same time. Both robots can work togather.


------------------------------------------ Nodes and their details---------------------------


There are 5 Main nodes running on moveit.launch.py file.

Node 1) Hand_tracking - Starts the camera, detects hand, generated landmark points, publishes  x and y POSESTAMPED msgs to a topic.
Node 2) teleop_moveit - Subscribes to the POSESTAMPED msgs, converts them into twist_msgs for Moveit_Servo to use.
Node 3) Moveit_servo - Servoing node which takes care of motion planning. The output will generate delta TwistCommands to the controllers.
Node 4) move_group - Motion planning node, reponsible for controllers. Controllers will act on delta twist commands and the robot will move.
Node 5) rviz2 - Visualisation Node.

------------------------------------------Package List------------------------------------------

1) franka_desrption - contains robot description files(i.e. URDF)
2) robot_bringup - contains hand_tracking and teleop_moveit nodes.
3) moveit_panda - contains robot controllers, SRDF , Kinematics file servo config and main launch file named 'moveit_rviz.launch.py'

------------------------------------------External Libraries/Packages---------------------------

1) Mediapipe
2) OpenCv2
3) ros2_control / ros2_controllers
4) Moveit


-------------------------------------------Refrences----------------------------------------------


Franka description - Stripped from Franka_ros2 - https://github.com/frankaemika/franka_ros2
MediaPipe - https://github.com/prashver/hand-landmark-recognition-using-mediapipe/blob/main/video_input/hand_tracking_video.py
Moveit_servo refrence - https://github.com/moveit/moveit2/tree/cc635471aadfb9446398ece319ae31c6b72bec86/moveit_ros/moveit_servo

other refrences(username) : 

Mediapipe(prashver) : https://github.com/glorie-git/gesture_recognizer
Mediapipe(Sousannah) : https://github.com/Sousannah/hand-tracking-using-mediapipe/blob/main/hand_tracking.py
Moveit_servo(horatioh13) : https://github.com/horatioh13/ROS_WORK 
Teleoperation a robot arm with computer vision (blog: Graham clifford)- used for methodology- https://graham-clifford.com/Robot-Arm-Teleoperation-Through-Computer-Vision-Hand-Tracking/









