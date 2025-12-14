# X2 LiDARBot Maze Solver

An autonomous maze-solving robot built with **M5Stack** and an **X2 LiDAR sensor**, implementing a right-hand wall-following algorithm with intelligent path detection.

---

## Hardware Requirements

- **M5Stack Core** — Main controller  
- **X2 LiDAR Sensor** — 360° distance sensing (Serial1, pins 16/17)  
- **LidarCar Base** — Motor control platform  

---

## Features

### Core Functionality

- **Autonomous Maze Navigation** — Right-hand wall-following algorithm  
- **360° LiDAR Mapping** — Real-time environment visualization on M5Stack display  
- **Reflective Exit Detection** — Automatically detects maze completion using reflective markers  
- **Data Filtering** — Interpolation algorithm removes noise and invalid readings  

### Navigation Logic

The robot prioritizes paths in the following order:

1. **Right turn** (when available)  
2. **Forward** (if right is blocked)  
3. **Left turn** (if right and forward are blocked)  
4. **U-turn** (dead end)  

---

## Control Modes

### Manual Control

Navigate manually using keyboard commands:

- `W` — Forward  
- `S` — Backward  
- `A` — Turn left  
- `D` — Turn right  
- `X` — Stop  

### Maze Mode

- `M` — Toggle autonomous maze solving on/off  
- Robot automatically navigates using the wall-following algorithm  

### Calibration Mode

- `C` — Enter turn calibration mode  

  - `1` — Test right 90° turn  
  - `2` — Test left 90° turn  
  - `3` — Test 180° turn  
  - `+` — Increase turn duration by 100 ms  
  - `-` — Decrease turn duration by 100 ms  
  - `S` — Show current calibration values  
  - `Q` — Exit calibration mode  

---

## Configuration Parameters

### Detection Thresholds

    threshold_front = 105.0 mm     // Minimum distance to consider front path open
    threshold_sides = 150.0 mm     // Minimum distance for left/right openings

### Turn Durations (calibrate for your robot)

    turnDuration90Right = 2200 ms
    turnDuration90Left  = 2200 ms
    turnDuration180     = 4300 ms

### LiDAR Scanning Zones

- **Front**: 155° – 195° (indexes 310–390)  
- **Left**: 70° – 116.5° (indexes 140–233)  
- **Right**: 252.5° – 290° (indexes 505–580)  

### Exit Detection

- Detects reflective surfaces with readings **greater than 4000 mm**  
- Requires **15+ consecutive readings**  
- Needs **two consecutive detection cycles** to confirm exit  

---

## Algorithm Details

### Data Cleaning

The `cleanLidarData()` function interpolates invalid readings using adjacent valid data points within a two-index gap.  
Values outside the **80–8000 mm** range are filtered out to reduce noise.

### U-Turn Prevention

After executing a **right turn** or **U-turn**, the robot ignores right-turn opportunities for **two seconds** to prevent immediate backtracking.

### Maze Exit Detection

Reflective tape or markers are placed at the maze exit.  
When the LiDAR detects highly reflective surfaces (>4000 mm) across the front scanning zone, the robot confirms maze completion and stops.

---

## Display Features

- **Real-time LiDAR visualization** — Polar plot showing detected obstacles  
- **Robot position** — Red dot at the center of the display  

---

## Getting Started

1. Upload the code to your **M5Stack**
2. Run **calibration mode (`C`)** to fine-tune 90° and 180° turns
3. Position the robot at the maze entrance
4. Press **`M`** to start autonomous solving
5. The robot stops automatically when the reflective exit is detected

---

## Technical Architecture

### Serial Communication

- **Serial** (115200) — Debug output and keyboard control  
- **Serial1** (115200, pins 16/17) — X2 LiDAR data  
- **Serial2** (115200) — Motor control commands  

---

## Maze Design Requirements

- Wall separation:
  - Front: >105 mm  
  - Sides: >150 mm  
- **Exit marker**: Reflective tape or reflective surface at maze exit  
- Recommended: Smooth, continuous walls for consistent LiDAR readings  

---

## Troubleshooting

**Robot does not turn accurately**

- Run calibration mode (`C`) and adjust turn durations  

**Fails to detect openings**

- Verify threshold values match maze dimensions  
- Confirm LiDAR functionality (use distance print mode if available)  

**Exit not detected**

- Ensure reflective marker is present and visible in front scanning zone  
- Adjust `REFLECTIVE_THRESHOLD` (default: 4000 mm)  

**Erratic behavior**

- Verify Serial1 wiring (pins 16/17)  
- Confirm LidarCar is receiving commands via Serial2  

---

## PlatformIO Unit Testing

This directory is intended for **PlatformIO Test Runner** and project tests.

Unit testing verifies individual MCU program modules, control logic, and data handling routines to ensure correctness and reliability.

More information:
https://docs.platformio.org/en/latest/advanced/unit-testing/index.html
