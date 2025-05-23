# 🤖 Multi-Arm Teleoperation with Hand-Tracking via MediaPipe ✋📷

The basic Idea of this project is to do the `teleopration` in cartesian space by moving the end effector  
in a desired position in x and Y. Since the `Depth Camera` is not available, Z is not used; limiting DOF of the  
robots to 3.

---

## 🛠️ Setup

The setup includes two 6 DOF `Franka_eamika research 3` robots placed at 0.75m distance facing each other in x axis.  
The 7th Revolute joint in `Franka` is set to fixed in URDF to mimic `6 DOF`. 

---

## ⚙️ Requirements

### ROS 2 Setup:

To run the setup, You need to have ROS2 Humble setup with Moveit2 motion planning working.  
Make sure to download all the controllers and additional packages that are listed in package.xml. 

The important one is `Joint Trajectory Controller`:

```bash
sudo apt update && sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers 
```

---

### 🧠 Hand Tracking Setup

From pip, please install mediapipe by executing:

```bash
pip install --upgrade pip
pip install mediapipe opencv-python
```

---

## 🕹️ Steps to Teleoperate 

### 1️⃣ Clone the repo and build ROS 2 workspace

```bash
git clone https://github.com/dhruvil122/hand_teleop.git
cd hand_teleop
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

---

### 2️⃣ Setup the Camera Feedback

#### 📷 Case 1) Do not have a direct Camera access?

Use the following command to run a Flask-based stream server:

```bash
python3 src/robot_bringup/scripts/flask_bringup.py 
```

Then access the feed at `http://$IPAddress:5000/video_feed` (replace `$IPAddress` with your `ifconfig` IP).

#### 🎥 Case 2) Have direct access to the camera?

Just comment out line no. 59 and uncomment line no. 60 in the hand tracking node from robot_bringup package — the camera setup should work.

---

### 3️⃣ Launch Full System (MoveGroup, RViz, Hand Tracking, Teleoperation Node)

```bash
ros2 launch moveit_panda moveit_rviz.launch.py arm_id:=multi_arm load_gripper:=true
```

---

### 4️⃣ Start Moving Hands!

Wait for 10 seconds for `moveit_servo` to activate. Then use both hands to control both arms simultaneously. Magic! ✨

---

## 📦 Nodes and their Roles

| Node              | Description                                                                 |
|-------------------|-----------------------------------------------------------------------------|
| `hand_tracking`   | Starts the camera, detects hand, publishes POSESTAMPED (x, y) messages.     |
| `teleop_moveit`   | Converts POSESTAMPED messages to Twist messages for MoveIt Servo.           |
| `moveit_servo`    | Computes and publishes joint commands based on Twist deltas.                |
| `move_group`      | Handles planning, controller routing, and collision checking.               |
| `rviz2`           | 3D visualisation of robot state and planning feedback.                      |

---

## 📁 Package List

- `franka_description`: Robot description files (URDF)
- `robot_bringup`: Includes hand tracking and teleoperation nodes
- `moveit_panda`: Controllers, SRDF, kinematics, servo configs, and the main launch file

---

## 🔌 External Dependencies

- Mediapipe  
- OpenCV2  
- `ros2_control` / `ros2_controllers`  
- MoveIt 2

---

## ⚠️ Limitations

1. Frame delay can happen due to hardware or image capture rate.
2. Risk of hitting kinematic **singularities** (since robot operates in 3 DOF only).

---

## 📚 References

- [Franka description (from franka_ros2)](https://github.com/frankaemika/franka_ros2)
- [MediaPipe Snippet (prashver)](https://github.com/prashver/hand-landmark-recognition-using-mediapipe/blob/main/video_input/hand_tracking_video.py)
- [MoveIt Servo Setup](https://github.com/moveit/moveit2/tree/cc635471aadfb9446398ece319ae31c6b72bec86/moveit_ros/moveit_servo)
- More contributors and ideas:
  - [glorie-git (gesture recognizer)](https://github.com/glorie-git/gesture_recognizer)
  - [Sousannah (hand tracking)](https://github.com/Sousannah/hand-tracking-using-mediapipe/blob/main/hand_tracking.py)
  - [horatioh13 (MoveIt + Servo)](https://github.com/horatioh13/ROS_WORK)
  - [Graham Clifford (CV-based Teleop Methodology)](https://graham-clifford.com/Robot-Arm-Teleoperation-Through-Computer-Vision-Hand-Tracking/)
