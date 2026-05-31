#  5-DoF Vision-Guided Robotic Arm: End-to-End Pipeline

This repository contains a full-stack robotics system integrating **Computer Vision**, **ROS2 middleware**, and **Embedded Inverse Kinematics**. The project enables a 5-Degree of Freedom (DoF) manipulator to autonomously perceive, track, and pick-and-place objects based on real-time spatial data.

---

##  System Architecture

The project follows a distributed control architecture, separating high-level perception from low-level hardware execution.

| Layer | Responsibility | Key Technologies |
| :--- | :--- | :--- |
| **Perception** | ArUco detection & Perspective Mapping | Python, OpenCV, NumPy |
| **Coordination** | Node communication & Command Filtering | ROS2 (Humble/Foxy), Serial |
| **Execution** | Inverse Kinematics (IK) & PWM Control | ESP32 (C++), PCA9685 |
| **Digital Twin** | Kinematic Validation & Path Tracing | MATLAB |

---

##  Computer Vision & Perception (ROS2)

The perception stack transforms 2D camera pixels into 3D robot workspace coordinates using a specialized vision pipeline.

* **ArUco Tracking:** The `ArucoDetectorNode` identifies unique IDs for workspace boundaries (corners) and target payloads.
* **Perspective Transform:** A $3 \times 3$ transformation matrix corrects for camera tilt and lens distortion, mapping the raw feed to a physical **15cm x 10cm** grid.
* **Hysteresis & Filtering:** The `ArucoSubscriber` node implements a **0.5cm** movement threshold to filter out camera noise and prevent servo jitter.
* **Command Cooldown:** A 6-second software lock ensures the robot completes its pick-and-place sequence before accepting a new target.

---

##  Kinematic Modeling

The arm utilizes a custom geometric **Inverse Kinematics (IK)** solver to translate Cartesian $(X, Y, Z)$ targets into joint angles.

### Mathematical Logic
The base rotation ($q_1$) is derived from:
$$q_1 = \text{atan2}(Y, X)$$

The elbow angle ($q_3$) is determined via the Law of Cosines to reach the distance $D$:
$$\cos(q_3) = \frac{D^2 - L_1^2 - L_{\text{eff}}^2}{2 \cdot L_1 \cdot L_{\text{eff}}}$$

*Note: $L_{\text{eff}}$ combines the forearm, wrist, and gripper lengths ($L_2 + L_3 + L_g$) into a single effective link for simplified planar calculation.*

### Safety & Vertical Alignment
* **Auto-Leveling:** The wrist ($q_5$) automatically adjusts its pitch to keep the gripper perpendicular to the work surface.
* **Z-Squash Function:** A software safety layer prevents the arm from colliding with the floor, squashing $Z$ inputs below **12cm** into a safe parabolic curve.
* **Safe Sweeps:** Movements are sequenced (Elbow → Base → Shoulder) to ensure the arm lifts clear of obstacles before rotating.

---

##  MATLAB Digital Twin

The `5-DOF_RRRRR_Arm.m` script serves as a verification environment. It maps the physical robot using parameters extracted from the `urdfnew.urdf` file.

* **Path History:** Visualizes the end-effector trajectory in 3D space to detect potential collisions.
* **Sinusoidal Easing:** Simulates smooth motion profiling to reduce mechanical stress using:
    $$\text{easing} = \frac{1 - \cos(\pi \cdot \text{step} / \text{total\_steps})}{2}$$
* **Accuracy Check:** Calculates the Euclidean distance between the IK solution and the intended target to verify model precision.

---

##  Hardware Specifications
* **Controller:** ESP32 (WROOM-32) @ 115200 Baud.
* **Actuation:** PCA9685 16-Channel 12-bit PWM driver.
* **Servos:** 6 High-torque servos (Base, Shoulder, Elbow, Wrist Roll, Wrist Pitch, Gripper).
* **Power:** External 5V/10A DC supply.

---

##  Execution Guide

### 1. Embedded Setup
Upload the `Main.cpp` firmware to your ESP32. Ensure the `channelMap` array matches your physical PCA9685 wiring (default pins: 0, 3, 4, 7, 8, 15).

### 2. ROS2 Workspace
Launch the nodes in separate terminals:
```bash
# 1. Start Vision Tracking
ros2 run my_arm_pkg aruco_detector

# 2. Start the ESP32 Bridge
ros2 run my_arm_pkg esp32_bridge --ros-args -p port:=/dev/ttyUSB0

# 3. Start the Coordinator
ros2 run my_arm_pkg aruco_subscriber
