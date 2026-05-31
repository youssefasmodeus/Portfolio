# PRPP Robot: Kinematics & Control Suite

This repository contains a comprehensive implementation of a 4-DOF **PRPP** (Prismatic-Revolute-Prismatic-Prismatic) robotic manipulator. This suite includes a high-fidelity **MATLAB Simulation** for workspace visualization and a **C++ Core** for algorithmic implementation and hardware integration.

---

##  System Architecture

The PRPP robot is designed with a unique sliding-base and radial-arm configuration:
* **Joint 1 (P):** Prismatic slider along the X-axis (Base).
* **Joint 2 (R):** Revolute joint rotating about the Z-axis.
* **Joint 3 (P):** Prismatic slider along the Z-axis (Elevation).
* **Joint 4 (P):** Prismatic radial extension (End-effector reach).

---

##  Kinematic Specifications

### Forward Kinematics (FK)
The End-Effector position $(X, Y, Z)$ is derived from joint states $(q_1, q_2, q_3, q_4)$ as follows:

$$X = q_1 + q_4 \cos(q_2)$$
$$Y = q_4 \sin(q_2)$$
$$Z = Z_{\text{offset}} + q_3$$

> **Fixed Offset:** $Z_{\text{offset}} = 0.220\text{ m}$ (Height of the base assembly).

### Joint & Workspace Limits
| Parameter | Type | Unit | Range [Min, Max] |
| :--- | :--- | :--- | :--- |
| **q1** | Prismatic | mm | [0, 190] |
| **q2** | Revolute | deg | [-180, 180] |
| **q3** | Prismatic | mm | [0, 400] |
| **q4** | Prismatic | mm | [0, 400] |
| **X-Reach** | Cartesian | mm | [-400, 590] |
| **Y-Reach** | Cartesian | mm | [-400, 400] |
| **Z-Reach** | Cartesian | mm | [220, 620] |

---

##  Software Components

### 1. MATLAB Simulation (`inv_kinematics.m`)
An interactive GUI environment used to prototype motions and verify reachability.
* **Shortest-Path Optimization:** Automatically selects the IK solution with the minimum Euclidean joint distance from the current state.
* **Visual Feedback:** Features a 3D robot model, path tracing, and real-time coordinate labels.
* **Validation:** Compares the analytical FK model against MATLAB's `rigidBodyTree` to ensure <1mm accuracy.

### 2. C++ Implementation (`main.cpp`)
The core computational engine designed for real-time control.
* **Efficient IK Solver:** Implements `atan2` for robust rotation calculation and Pythagorean logic for radial reach.
* **Safety Guards:** Logic to prevent "out-of-bounds" joint commands and handle singular configurations.
* **Modular Design:** Separates kinematic logic from I/O, allowing for easy porting to embedded platforms (Arduino, STM32, or ROS nodes).

---

##  Execution Guide

### Running the MATLAB Simulation
1.  Ensure **Robotics System Toolbox** is installed.
2.  Run `inv_kinematics.m`.
3.  Enter target coordinates $(X, Y, Z)$ in **mm** when prompted in the Command Window.
4.  The robot will animate to the target using a sequential joint motion strategy.

### Compiling the C++ Interface
```bash
# Compile using GCC
g++ main.cpp -o prpp_control

# Run the executable
./prpp_control
