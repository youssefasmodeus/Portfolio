# Wireline Depth Acquisition System — SLB R&D Testbench

A deterministic **Simulink/Simscape** simulation of an industry-standard wireline depth control system, built to validate the three layers of the surface acquisition depth algorithm used in oilfield wireline logging operations.

---

## Overview

Wireline depth measurement is one of the most critical challenges in oilfield logging. The measured depth of a formation directly determines where perforations are placed, where casing is set, and how logs are correlated across wells. A depth error of even 0.5 meters can result in a missed pay zone or a misplaced perforation costing millions of dollars.

This project builds a clean, physically accurate simulation testbench that models the full surface acquisition chain — from the winch drum paying out cable to the depth algorithm processing encoder pulses — and validates three correction layers that the industry uses to achieve depth accuracy better than ±0.1%.

The model deliberately avoids modeling chaotic downhole stick-slip phenomena. The physical plant is a smooth, predictable **Variable Stiffness Model** that isolates and validates the surface algorithm layers independently.

---

## Algorithm Layers Validated

### 1. Fastest-Wheel Logic (Micro-correction)
Based on **Schlumberger Patent US4924596** and the **Halliburton WSDP Panel AMS4A043** specification.

Two measuring wheels ride on the wireline cable at surface. Either wheel can momentarily lose contact with the cable due to mud contamination, cable vibration, or surface debris. A slipping wheel under-reads cable displacement. The algorithm reads both encoders every 10ms and selects the wheel that moved furthest — the faster wheel is always the one maintaining cable contact.

- Per-sample displacement comparison: `δᵢ(t) = xᵢ(t) − xᵢ(t − Δt)`
- Wheel selection: `active = argmax |δᵢ|`
- Slip detection threshold: LV = 0.5 inch (0.0127m) per sample
- Cumulative slip alarm: 4 inches (0.1016m)

### 2. RULC — Rig-Up Length Correction (Static offset)
A one-time static correction applied at the start of each logging run. The surface rig-up length (cable between drum and wellhead) and the downhole rig-up length (cable between cable head and top of tool) are physically different. RULC corrects the depth counter for this offset. Set to zero in this simulation as it is a field measurement irrelevant to dynamic validation.

### 3. SCORR — Stretch Correction (Dynamic offset)
Based on the **Lubinski stretch formula**. The wireline cable is elastic — under its own weight and the tool weight, it stretches. This stretch grows with depth and causes the surface measurement to overestimate true tool depth.

$$\delta L = \frac{F_{\text{tension}} \times L_0}{EA}$$

where:

$$F_{\text{tension}} = W_{\text{tool}} + \lambda_{\text{eff}} \cdot g \cdot L_0$$

The corrected true depth is:

$$\text{True Depth} = \text{depth}_{\text{wheel}} - \delta L$$

---

## Physical Model

The simulation models the complete mechanical system using **Simscape** for the physical domain and **Simulink** for the signal processing domain.

### Cable Mechanics
The wireline cable is modeled as a variable-stiffness elastic element where stiffness decreases as more cable is deployed:

$$k(t) = \frac{EA}{L_0(t)}$$

The deployed unstretched length is integrated from surface velocity:

$$L_0(t) = L_{0,\text{init}} + \int_0^t v_s(\tau)\, d\tau$$

The force applied to the tool mass combines elastic restoring force and cable self-weight:

$$F_{\text{total}} = F_{\text{stretch}} + F_{\text{weight}} = \frac{EA \cdot \delta L}{L_0} + \lambda_{\text{eff}} \cdot g \cdot L_0$$

### Tool Dynamics
$$m_t \ddot{x}_t = F_{\text{total}} - c_t \dot{x}_t$$

where $c_t$ is the viscous mud drag coefficient (no static friction — smooth plant by design).

### Natural Frequency
The cable-tool system has a depth-dependent natural frequency:

$$\omega_n(\bar{L}) = \sqrt{\frac{EA}{m_t \bar{L}}}$$

At 3000m with typical wireline parameters: $\omega_n \approx 0.4$ Hz — within the logging acquisition bandwidth.

---

## Repository Structure

```
IntegratedDepthWheel/
│
├── IntegratedDepthWheel.slx          # Main Simulink model
│
├── subsystems/
│   ├── Surface_Winch_Drum            # Drum + Ideal Angular Velocity Source
│   ├── Cable_System                  # Physical plant (cable + tool)
│   │   ├── Cable_Stretch_Calculator  # Lubinski stretch computation
│   │   ├── Cable_Weight_Force        # Ideal Force Source
│   │   ├── Motion_Sensor_Surface     # vs(t), xs(t) measurement
│   │   └── Tool_Mass                 # Mass + translational damper
│   ├── Quadrature_Encoders           # Measuring head + slip injection
│   └── True_Depth                   # SCORR correction chain
│
├── fastest_wheel.m                   # MATLAB Function — depth algorithm
│
├── dashboard/
│   └── index.html                    # Standalone web validation dashboard
│
└── README.md
```

---

## Simulink Model Architecture

```
[Input Speed] ──► [Surface Winch & Drum]
                          │ mechanical
                          ▼
              [Cable Weight + Variable Force] ◄── F_Total ◄── [Cable Stretch Calculator]
                          │                                            ▲
                          ├──► [Motion Sensor Surface] ──── Vs ───────┘
                          │
                          └──► [Tool Mass]
                                    └──► [Motion Sensor For Tool] ──── Xt (ground truth)

[Quadrature Encoders] ──► pos1, vel1, pos2, vel2
                                    │
                         [Fastest Wheel Algorithm]
                                    │
                    depth, depth_rate, active_wheel, slip_alarm
                                    │
                            [True Depth] ◄── delta_L
                                    │
                              True_Depth (output)
```

---

## Fastest Wheel Algorithm

```matlab
function [depth, depth_rate, active_wheel, slip_alarm, slip_magnitude] = ...
         fastest_wheel(pos1, vel1, pos2, vel2)

% Parameters
switch_threshold = 0.05;   % m/s — deadband to prevent noise-driven switching
LV               = 0.005;  % m   — slip detection threshold per sample
slip_limit       = 0.1016; % m   — cumulative alarm limit (4 inches)

% Layer 1: Per-sample position deltas
delta1 = pos1 - pos1_prev;
delta2 = pos2 - pos2_prev;

% Layer 2: Fastest wheel selection (velocity-based with deadband)
vel_diff = abs(vel1) - abs(vel2);
if     vel_diff >  switch_threshold, active_wheel_state = 1;
elseif vel_diff < -switch_threshold, active_wheel_state = 2;
end

% Layer 3: Depth accumulation from fastest wheel
depth_accum = depth_accum + chosen_delta;

% Layer 4: Slip detection and alarm
% Registers events regardless of algorithm correction (data quality flag)
if slip_event && delta_diff > LV
    slip_counter = slip_counter + delta_diff;
end
```

---

## Validation Test Sequence

A controlled slip injection sequence validates all algorithm layers:

| Time | Wheel 1 | Wheel 2 | Expected Response |
|------|---------|---------|-------------------|
| 0 – 35s | Clean (1.0) | Clean (1.0) | Baseline — both wheels agree |
| 35 – 50s | Slip (0.8) | Clean (1.0) | Algorithm switches to Wheel 2 |
| 50 – 55s | Clean (1.0) | Clean (1.0) | Recovery period |
| 55 – 70s | Clean (1.0) | Slip (0.6) | Algorithm stays on Wheel 1 |
| 70 – 100s | Clean (1.0) | Clean (1.0) | Full recovery |

### Validation Results

| Metric | Result | Physical Meaning |
|--------|--------|-----------------|
| Wheel selection | Never selected slipping wheel | Algorithm correctly rejects under-reading wheel |
| Depth continuity | No jumps at any transition | Depth accumulation protected during slip events |
| Max SCORR stretch at 200m | ~9 cm | Physically correct per Lubinski formula |
| Net final depth error | < 1 cm | SCORR correction successfully applied |
| Slip alarm | Triggers during slip, decays after | Data quality flag working correctly |

---

## Model Parameters

| Parameter | Value | Physical Meaning |
|-----------|-------|-----------------|
| `EA` | 10 × 10⁶ N | Cable axial stiffness (E × cross-sectional area) |
| `W_tool_g` | 2943 N | Tool weight (300 kg × 9.81) |
| `lambda_eff_g` | 7.85 N/m | Buoyed cable linear weight density |
| `L0_init` | 100 m | Initial deployed cable length (rig-up) |
| `r_wheel` | 0.0508 m | Measuring wheel radius (2 inch standard) |
| `r_drum` | 0.4 m | Winch drum radius |
| `RULC` | 0 m | Rig-up length correction (zero in simulation) |

### Solver Configuration

```
Solver:         ode3 (Bogacki-Shampine, fixed-step)
Step size:      0.01 s
Stop time:      100 s
Simscape:       Fixed-cost, 3 nonlinear iterations
```

---

## Web Dashboard

A standalone interactive validation dashboard (`dashboard/index.html`) runs the fastest wheel algorithm and Lubinski stretch correction entirely in the browser with no MATLAB required.

**Features:**
- Live depth gauges — True Depth, Raw Depth, SCORR Stretch, Slip Alarm
- Numeric parameter inputs for all physics and simulation settings
- Configurable slip injection with independent start time, end time, and factor per wheel
- Live Chart.js plots — depth comparison, stretch correction, wheel velocities
- Wheel status indicators — ACTIVE / SLIPPING / STANDBY per wheel
- Algorithm event log with timestamped wheel switches and alarm events
- Automatic error metrics — max slip error, RMS error, net final error, stretch/depth ratio
- Exportable validation report as a plain text file

**To run:** Open `dashboard/index.html` in any browser. No server, no dependencies, no installation.

---

## Industry References

1. **Schlumberger Patent US4924596** — *Depth Measurement System* — defines the fastest-wheel algorithm, LV threshold (0.5 inch), and cumulative slip alarm (4 inches)

2. **Halliburton WSDP Panel AMS4A043 Rev.H** (Benchmark Wireline Products, 2010) — field implementation specification: 10ms sample rate, dual encoder selection, PPF calibration

3. **Lubinski, A.** — *A Study of the Buckling of Rotary Drilling Strings* — foundation of the elastic stretch correction formula used in all modern wireline SCORR algorithms

4. **API RP 31B** — *Measurement of Well Depth* — industry standard specifying depth accuracy requirements for wireline operations

---

## Requirements

- MATLAB R2024a or later
- Simulink
- Simscape
- Simscape Driveline
- No additional toolboxes required for the web dashboard

---

## Key Design Decisions

**Why Ideal Rotational Motion Sensors instead of Incremental Shaft Encoders:**
The Simscape Driveline Incremental Shaft Encoder produces pulse signals at the physical domain boundary that cannot be cleanly converted to Simulink signals. The Ideal Rotational Motion Sensor provides continuous angular velocity and position directly, eliminating boundary noise while preserving all physically relevant behaviors — wheel inertia, rotational damping, and slip dynamics.

**Why Lubinski formula instead of geometric stretch:**
In a real logging operation, tool position is never directly measured — only surface cable payout is known. The Lubinski formula computes stretch entirely from surface measurements (cable speed, cable weight, tool weight, cable stiffness), which is exactly how real acquisition systems compute the SCORR correction. Using the Simscape tool position `xt` as a stretch reference would not match field practice and would require sensor reference frame corrections.

**Why smooth plant (no stick-slip):**
Downhole stick-slip introduces chaotic, broadband vibration into the cable system that masks the deterministic stretch signal. By using a viscous damper with no static friction, the plant produces clean, predictable behavior that lets the algorithm validation focus on depth accuracy rather than vibration rejection.

---

## License

This project was developed as an independant private project. All physical models, algorithm implementations, and validation methods are based on publicly available patents and industry specifications referenced above.
