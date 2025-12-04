# Changelog

## 2025-12-02

### Camera Calibration System
- Created `calibration/calibrate.py` - standalone script for one-time camera calibration
  - Uses 5x8 checkerboard pattern (43mm squares, 4x7 inner corners)
  - ASCII terminal UI (no GUI dependencies)
  - Auto-captures 7-15 stable checkerboard poses
  - Saves calibration to `calibration/camera_config.json`
  - Computes camera matrix and lens distortion coefficients

### Wind Speed Estimator Updates (`wind_speed_estimator.py`)
- Added calibration loading and lens distortion correction
  - `load_calibration()` - loads config and creates undistortion maps
  - `undistort_frame()` - applies lens correction using pre-computed maps
  - Calibration automatically loaded on startup if available

- Fixed terminal ASCII display
  - Camera Preview (left): Shows raw uncalibrated frame, no overlays
  - Green Detection (right): Shows calibrated frame with mask overlay and markers
  - Removed text overlays from terminal view (only in video output)

- Added calibrated video output
  - Now saves `calibrated.mp4` in addition to `raw.mp4` and `video.mp4`
  - Output files:
    - `data.csv` - timestamped measurements
    - `video.mp4` - processed video with overlays and plot
    - `raw.mp4` - raw camera feed (uncalibrated)
    - `calibrated.mp4` - calibrated video (lens distortion corrected)
    - `ascii_video.txt` - terminal visualization log

- Reduced salt-and-pepper noise in terminal ASCII view
  - Added median filter (5x5) before downscaling - removes outlier pixels
  - Increased Gaussian blur from 5x5 to 9x9 for smoother output

### Notebook Updates (`angle_detection_notebook.ipynb`)
- Added Section 2.5: Load Camera Calibration
  - Loads calibration from `calibration/camera_config.json`
  - Creates undistortion maps for fast per-frame correction
  - Sets `calibration_enabled` flag

- Updated Section 4 (Test Detection)
  - Applies `undistort_frame()` when calibration available
  - Shows "(Calibrated)" in plot title when active

- Updated Section 5 (Live Detection)
  - Applies lens distortion correction to each frame
  - Shows `[CAL]` or `[UNCAL]` status in video overlay

### Documentation
- Updated `README.md` with calibration instructions and system design
