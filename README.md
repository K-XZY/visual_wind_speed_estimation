# Wind Speed Estimation via Pole Angle Detection

> **Note:** This folder (`CW_Solution/`) has its own git repository separate from the outer tutorial repo.
> - **Push to:** https://github.com/K-XZY/visual_wind_speed_estimation (origin)
> - **NOT to:** the outer Dkaka/COMP0219_Tutorials repo
>
> If you cloned from the outer tutorial repository, always use the code in `CW_Solution/`.

## Project Overview

This project implements a real-time computer vision algorithm to estimate wind speed by measuring the deflection angle of a hanging pole with green tape markers against a checkerboard pattern background.

## System Components

### Scene Setup
- **Hanging pole** - Hangs vertically when no wind is present, deflects horizontally with wind
- **Green tape marker** - Located near the middle of the pole for tracking
- **Checkerboard pattern** - Background for lens distortion correction and reference frame
- **Camera** - Raspberry Pi camera capturing the scene

### Algorithm Objective
Detect and track the pole's deflection angle in real-time as an indicator of wind speed. The angle measurement provides an estimate of the force acting on the pole, which correlates with wind velocity.

## Quick Start

### 1. Camera Calibration (One-time Setup)

Before running the wind speed estimator, calibrate the camera to correct lens distortion:

```bash
cd CW_Solution/calibration
python calibrate.py
```

**Calibration Instructions:**
1. Hold the 5x8 checkerboard (43mm squares) in front of the camera
2. Move it to different positions and angles within the frame
3. Hold still when prompted - the script auto-captures stable frames
4. After 15+ captures, calibration is computed and saved automatically

The calibration is saved to `calibration/camera_config.json` and will be automatically loaded by the wind speed estimator.

### 2. Run Wind Speed Estimator

```bash
cd CW_Solution
python wind_speed_estimator.py
```

Press `Ctrl+C` to stop recording and save files.

## Project Structure

```
CW_Solution/
├── wind_speed_estimator.py    # Main application
├── calibration/
│   ├── calibrate.py           # Camera calibration script
│   └── camera_config.json     # Saved calibration (auto-generated)
├── output/                    # Recording outputs (auto-generated)
│   └── {timestamp}/
│       ├── data.csv           # Timestamped measurements
│       ├── video.mp4          # Processed video with overlays
│       ├── raw.mp4            # Raw camera feed
│       └── ascii_video.txt    # Terminal visualization log
└── README.md
```

## Technical Requirements

### Angle Measurement
- **Range:** 0 to π/2 radians (0° to 90°)
- **Reference:** Absolute angle from vertical
  - 0 rad (0°) = Pole hanging straight down (no wind)
  - π/2 rad (90°) = Pole completely horizontal (maximum wind)

### System Constraints
- **Real-time processing** required
- Modular implementation for testing individual components

## Technical Approach

### Pipeline Architecture

1. **Camera Calibration Module** ✓
   - Use checkerboard pattern to calculate camera matrix and distortion coefficients
   - Calibration saved to JSON config file (one-time setup)
   - Undistortion maps pre-computed for fast per-frame correction

2. **Green Tape Detection Module** ✓
   - HSV color space segmentation
   - Contour detection and filtering
   - Centroid calculation for tape marker

3. **Angle Calculation Module** ✓
   - Determine pole orientation from marker position
   - Calculate angle from vertical using reference frame
   - Output angle in radians [0, π/2]

4. **Real-time Integration** ✓
   - Combine modules into real-time pipeline
   - Display live angle measurements
   - Performance optimization

### Calibration System Design

The calibration system uses a **separate calibration script** approach:

- **Why separate?** Camera calibration only needs to be done once (unless the camera/lens changes), so it's kept separate from the main application to avoid unnecessary complexity during normal operation.

- **Config persistence:** Calibration parameters are saved to `calibration/camera_config.json`, which contains:
  - Camera intrinsic matrix (focal length, principal point)
  - Lens distortion coefficients (radial and tangential)
  - Image resolution and calibration metadata
  - Reprojection error for quality assessment

- **Automatic loading:** The wind speed estimator automatically loads the calibration on startup. If no calibration exists, it runs without distortion correction and displays a warning.

### Checkerboard Specifications

- **Pattern:** 5×8 squares (4×7 inner corners)
- **Square size:** 43mm

### Technology Stack
- **Picamera2** - Video capture from Raspberry Pi camera
- **OpenCV** - Image processing and computer vision
- **NumPy** - Numerical computations
- **Python 3** - Implementation language

## Development Status

- [x] Camera calibration using checkerboard pattern
- [x] Green tape detection and segmentation (HSV color space)
- [x] Pole angle calculation from marker position
- [x] Real-time video processing pipeline
- [x] Angle visualization overlay
- [x] Wind speed estimation (linear model)
- [ ] Wind speed calibration curve refinement (angle → wind speed mapping)
- [ ] GUI integration for data visualization

## GUI Integration

The project integrates with the **GUI_Rev3** visualization tool located on a separate laptop. The GUI reads wind speed data over serial and displays real-time plots.

> **IMPORTANT: Do not modify the data output format of the wind speed estimator.**
>
> The GUI expects a specific serial data format and any changes to the output format will break compatibility. The GUI code in `GUI_Rev3/` is maintained separately and should not be modified as part of this project.

### Integration Status

- [x] Serial I/O module implemented (`serial_io.py`)
- [x] Ground truth reading support (SerialInput class)
- [x] Wind speed output support (SerialOutput class)
- [x] STM32 UART bridge firmware (`UART_streaming/`)
- [ ] **Phase 1:** Test GT sensor reading on Pi
- [ ] **Phase 2:** Test STM32 bridge (Pi → Laptop)
- [ ] **Phase 3:** Full demo day integration

See [TODO.md](TODO.md) for detailed next steps.

## Live Tuning System

### Overview

The system supports live parameter tuning by comparing our angle-based wind speed estimate against a ground truth sensor. This allows optimizing the conversion coefficients (α, β, γ) before the marking phase.

### Demo Day Workflow

**Phase 1 - Tuning:**
1. Connect ground truth sensor to Pi via USB
2. Update `GUI_Rev3/config.yaml` with correct port (run `ls /dev/tty*` to find it)
3. Run `python wind_speed_estimator.py`
4. Press 't' to start collecting paired data (angle measurements + ground truth)
5. Collect ~60 seconds of data across different wind speeds
6. Press 't' again to stop collection and run optimization
7. PSO finds best α, β, γ parameters → saved to `tuning/parameters.json`
8. Disconnect ground truth sensor from Pi

**Phase 2 - Marking:**
1. Wire Pi to STM32 (see below)
2. Connect STM32 USB to laptop
3. Run `python wind_speed_estimator.py` - it outputs wind speed via UART
4. Laptop runs GUI_Rev3 which reads from STM32 USB (`/dev/ttyACM0`)
5. Ground truth sensor connected directly to laptop
6. Both streams (your estimate + ground truth) recorded for comparison

### Hardware Setup

**STM32 acts as UART-to-USB bridge** (firmware in `UART_streaming/`):

```
Phase 1 (Tuning):
  GT Sensor ──USB──► Pi (/dev/ttyUSB0)
                     └── reads GT, runs PSO, saves params

Phase 2 (Marking):
  Pi GPIO14 (TX) ────► STM32 PA10 (D2/RX)
  Pi GND ────────────► STM32 GND
                         │
                         └──► STM32 USB ──► Laptop (/dev/ttyACM0)
                                            └── GUI reads wind speed

  GT Sensor ─────────────────────────────► Laptop (separate USB)
                                            └── GUI reads ground truth
```

**Wiring (Pi → STM32):**
| Pi Pin | Signal | STM32 Pin |
|--------|--------|-----------|
| Pin 8  | TX (GPIO14) | PA10 / D2 |
| Pin 6  | GND | GND |

**Cables needed:**
- 2x jumper wires: Pi TX→STM32, Pi GND→STM32
- USB cable: STM32 ↔ Laptop
- USB cable: GT sensor ↔ Pi (tuning) or Laptop (marking)
- Pi power cable
- Camera ribbon (internal)

### Data Flow

```
┌─────────────────────────────────────────────────────────────┐
│                    wind_speed_estimator.py                  │
│                                                             │
│  [Camera] → angle θ, θ̇                                      │
│                 ↓                                           │
│  [Serial In] ← ground truth (float per line, 115200 baud)  │
│                 ↓                                           │
│  [Tuning Mode]                                              │
│    - Press 't': start recording paired data                 │
│    - Press 't': stop & run particle swarm optimization      │
│    - Saves best α,β,γ to tuning/parameters.json             │
│                 ↓                                           │
│  [Serial Out] → wind speed (float per line) → Laptop GUI    │
└─────────────────────────────────────────────────────────────┘
```

### Configuration

Both the wind speed estimator and GUI share the same config file: `GUI_Rev3/config.yaml`

```yaml
devices:
  - name: Ground Truth
    port: /dev/ttyUSB0      # Ground truth sensor input (for tuning)
    baud: 115200
  - name: Student
    port: /dev/serial0      # Pi hardware UART (GPIO14 TX) → STM32 → Laptop
    baud: 115200
```

The estimator reads this config to know:
- Which port to read ground truth from (first device)
- Which port to output wind speed to (second device, via Pi UART → STM32)

### Parameters File

```
CW_Solution/
├── tuning/
│   ├── parameters.json    # Auto-generated after tuning
│   └── data/
│       └── {timestamp}.json  # Saved tuning data
```

Format for `parameters.json`:
```json
{
  "alpha": 1.0,
  "beta": 0.1,
  "gamma": 0.0
}
```

- If file exists: parameters loaded on startup
- If file missing: uses defaults (α=1.0, β=0.1, γ=0.0)
- After tuning: file updated with optimized values

### Tuning Behavior

**With ground truth connected:**
- Press 't' → start recording paired data
- Press 't' again → stop, save data to `tuning/data/{timestamp}.json`, run PSO optimization

**Without ground truth (shows "N/A"):**
- Press 't' → start recording (angle data only)
- Press 't' again → stop, save data only (no optimization)

### Tuning Algorithm

- **Method:** Particle Swarm Optimization (PSO)
- **Search space:** 3D (α, β, γ)
- **Objective:** Minimize MSE between `v = α*θ + β*θ̇ + γ` and ground truth
- **Data:** ~60 seconds of paired (θ, θ̇, ground_truth) samples

## Implementation Architecture

### Module Structure

```
CW_Solution/
├── wind_speed_estimator.py    # Main application (orchestrates modules)
├── serial_io.py               # NEW: Serial communication module
├── tuning/
│   ├── __init__.py
│   ├── parameters.py          # NEW: Parameter loading/saving
│   ├── pso.py                 # NEW: Particle swarm optimization
│   └── parameters.json        # Auto-generated config
├── calibration/
│   ├── calibrate.py
│   └── camera_config.json
└── output/
```

### Module Responsibilities

**1. `serial_io.py` - Serial Communication**
```python
class SerialInput:
    """Non-blocking reader for ground truth sensor."""
    def __init__(self, port: str, baud: int = 115200)
    def get_latest() -> float | None  # Returns latest value or None
    def close()

class SerialOutput:
    """Writer for STM32 output."""
    def __init__(self, port: str, baud: int = 115200)
    def send(value: float)  # Sends "value\n"
    def close()
```

**2. `tuning/parameters.py` - Parameter Management**
```python
@dataclass
class WindParams:
    alpha: float = 1.0
    beta: float = 0.1
    gamma: float = 0.0

def load_parameters() -> WindParams
def save_parameters(params: WindParams)
```

**3. `tuning/pso.py` - Optimization**
```python
@dataclass
class TuningDataPoint:
    theta: float
    theta_dot: float
    ground_truth: float

def optimize_parameters(data: list[TuningDataPoint]) -> WindParams
```

**4. `wind_speed_estimator.py` - Main (Modified)**
- Import and use new modules
- Add tuning state machine
- Integrate serial I/O into main loop
- Add keyboard input handling for 't' key

### Integration Points

```
wind_speed_estimator.py
    │
    ├── serial_io.SerialInput ─── reads ground truth (threaded)
    ├── serial_io.SerialOutput ── writes wind speed
    │
    ├── tuning.parameters ─────── load/save α,β,γ
    └── tuning.pso ────────────── optimize after data collection
```

### State Machine for Tuning

```
┌─────────────┐  press 't'  ┌─────────────┐  press 't'  ┌─────────────┐
│    IDLE     │ ──────────► │  RECORDING  │ ──────────► │  OPTIMIZING │
│             │             │  (collect)  │             │   (PSO)     │
└─────────────┘             └─────────────┘             └──────┬──────┘
       ▲                                                       │
       └───────────────────────────────────────────────────────┘
                              auto-return when done
```

### Implementation Order

1. **`tuning/parameters.py`** - Simple, no dependencies, enables testing
2. **`serial_io.py`** - Independent module, can test with mock serial
3. **`tuning/pso.py`** - Pure algorithm, testable with synthetic data
4. **Integrate into `wind_speed_estimator.py`** - Wire everything together
