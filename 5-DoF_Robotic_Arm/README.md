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

##  Computer Vision & Perception

The perception stack transforms 2D camera pixels into 3D robot workspace coordinates using a specialized vision pipeline.

- **ArUco Tracking:** The `ArucoDetectorNode` identifies unique IDs for workspace boundaries (corners) and target payloads.
- **Perspective Transform:** A $3 \times 3$ transformation matrix corrects for camera tilt and lens distortion, mapping the raw feed to a physical **15 cm × 10 cm** grid.
- **Hysteresis & Filtering:** The `ArucoSubscriber` node implements a **0.5 cm** movement threshold to suppress camera noise and prevent servo jitter.
- **Command Cooldown:** A 6-second software lock ensures the robot completes its full pick-and-place sequence before accepting a new target.

---

##  Kinematic Modelling

The arm uses a custom geometric **Inverse Kinematics (IK)** solver to translate Cartesian $(X, Y, Z)$ targets into joint angles.

### Joint Angle Derivation

**Base rotation** $q_1$ is computed directly from the target's horizontal position:

$$q_1 = \arctan2(Y,\ X)$$

**Elbow angle** $q_3$ is resolved via the Law of Cosines over the reach distance $D$:

$$\cos(q_3) = \frac{D^2 - L_1^2 - L_\text{eff}^2}{2 \cdot L_1 \cdot L_\text{eff}}$$

> **Link Model:** For planar IK calculation, the forearm, wrist, and gripper are collapsed into a single **effective link** $L_\text{eff} = L_2 + L_3 + L_g$. This simplifies the geometry to a standard 2-link planar problem without loss of accuracy for the target workspace.

### Safety & Vertical Alignment

- **Auto-Leveling:** The wrist joint $q_5$ automatically adjusts its pitch to keep the gripper perpendicular to the work surface at all times.
- **Z-Squash Function:** A software safety layer prevents floor collisions by remapping any $Z$ input below **12 cm** onto a safe parabolic curve.
- **Safe Sweep Sequencing:** Motions are executed in a fixed order — **Elbow → Base → Shoulder** — to guarantee the arm lifts clear of obstacles before rotating.

---

##  MATLAB Digital Twin

The `DoF5_RRRRR_Arm.m` script provides a simulation and verification environment. Robot geometry is sourced directly from `urdfnew.urdf` to keep the model consistent with the physical hardware.

- **Path History:** Renders the full end-effector trajectory in 3D space, making potential collisions or workspace violations immediately visible.
- **Sinusoidal Easing:** Motion profiles are shaped using a cosine ramp to reduce mechanical stress and replicate smooth servo behaviour:

$$\text{easing}(t) = \frac{1 - \cos\!\left(\pi \cdot \dfrac{t}{T}\right)}{2}$$

- **Accuracy Verification:** After each IK solve, the script computes the Euclidean error between the resolved end-effector pose and the intended target, confirming model precision.

---

##  Hardware Specifications

| Component | Details |
| :--- | :--- |
| **Microcontroller** | ESP32 (WROOM-32) @ 115200 baud |
| **PWM Driver** | PCA9685 — 16-channel, 12-bit |
| **Actuators** | 6 × high-torque servos (Base, Shoulder, Elbow, Wrist Roll, Wrist Pitch, Gripper) |
| **Power Supply** | External 5 V / 10 A DC |

---

##  Execution Guide

### 1. Embedded Setup

Upload `Main.cpp` to your ESP32. Verify that the `channelMap` array matches your physical PCA9685 wiring before flashing.

> Default channel mapping: `0, 3, 4, 7, 8, 15`

### 2. ROS2 Workspace

Launch each node in a separate terminal:

```bash
# Terminal 1 — Vision tracking
ros2 run my_arm_pkg aruco_detector

# Terminal 2 — ESP32 serial bridge
ros2 run my_arm_pkg esp32_bridge --ros-args -p port:=/dev/ttyUSB0

# Terminal 3 — Coordinator
ros2 run my_arm_pkg aruco_subscriber
```
