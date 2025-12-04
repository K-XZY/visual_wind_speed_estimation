#!/usr/bin/env python3
"""
Wind Speed Estimator

Estimates wind speed by measuring the deflection angle of a rigid pole
hanging as a pendulum from a fixed pivot point.

Two green markers on the pole are detected via camera, and the angle
from vertical is used to compute wind speed.

Formula: v_wind = α * √tan(θ) + β * θ̇ + γ

This is derived from pendulum physics where drag force balances gravity:
    tan(θ) = F_drag / F_gravity ∝ v²
    → v ∝ √tan(θ)

Output:
    - output/{timestamp}/data.csv - timestamped measurements
    - output/{timestamp}/video.mp4 - processed video with overlays and plot
    - output/{timestamp}/raw.mp4 - raw camera feed (uncalibrated)
    - output/{timestamp}/calibrated.mp4 - calibrated video (lens distortion corrected)

Usage:
    python wind_speed_estimator.py

Controls:
    - 't': Toggle tuning data collection
    - Ctrl+C: Stop recording and save files
"""

import numpy as np
import cv2
from picamera2 import Picamera2
import time
import os
import sys
import shutil
import json
import select
import termios
import tty
from datetime import datetime
from enum import Enum
import matplotlib.pyplot as plt
from matplotlib.backends.backend_agg import FigureCanvasAgg

# Import tuning and serial modules
from tuning import WindParams, load_parameters, save_parameters, TuningDataPoint, optimize_parameters
from tuning.pso import save_tuning_data
from serial_io import SerialInput, SerialOutput, load_serial_config


# =============================================================================
# TUNING STATE MACHINE
# =============================================================================

class TuningState(Enum):
    IDLE = "idle"
    RECORDING = "recording"
    OPTIMIZING = "optimizing"


# =============================================================================
# CONFIGURATION
# =============================================================================

# Wind speed conversion coefficients (defaults, overridden by tuning/parameters.json)
# v_wind = α * √tan(θ) + β * θ̇ + γ
DEFAULT_ALPHA = 15.0   # Coefficient for √tan(θ) term (m/s)
DEFAULT_BETA = 0.1     # Angular velocity coefficient (m/s per rad/s)
DEFAULT_GAMMA = 0.0    # Offset (m/s)

# HSV thresholds for green marker detection
HSV_LOWER_GREEN = np.array([35, 50, 50])
HSV_UPPER_GREEN = np.array([85, 255, 255])

# Minimum contour area for valid marker detection (pixels)
MIN_CONTOUR_AREA = 100

# Camera settings
CAMERA_RESOLUTION = (1280, 720)
CAMERA_FPS = 30

# Plot settings
PLOT_HEIGHT = 300  # pixels
PLOT_HISTORY_SECONDS = 10  # how many seconds of data to show in plot

# Uncertainty estimation settings
UNCERTAINTY_WINDOW_FRAMES = 30  # Rolling window size for σ estimation (~1 second at 30fps)

# Terminal display settings
TERMINAL_PREVIEW_WIDTH = 80  # Characters wide for camera preview
TERMINAL_PREVIEW_HEIGHT = 20  # Characters tall for camera preview

# Calibration config path
CALIBRATION_CONFIG_PATH = os.path.join(os.path.dirname(__file__), "calibration", "camera_config.json")


# =============================================================================
# CALIBRATION FUNCTIONS
# =============================================================================

def load_calibration() -> tuple:
    """
    Load camera calibration from config file and create undistortion maps.

    Returns:
        Tuple of (map1, map2) for cv2.remap(), or (None, None) if no calibration
    """
    if not os.path.exists(CALIBRATION_CONFIG_PATH):
        return None, None

    try:
        with open(CALIBRATION_CONFIG_PATH, 'r') as f:
            config = json.load(f)

        camera_matrix = np.array(config["camera_matrix"])
        dist_coeffs = np.array(config["distortion_coefficients"])
        image_size = tuple(config["image_size"])

        # Compute optimal new camera matrix
        new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
            camera_matrix, dist_coeffs, image_size, 1, image_size
        )

        # Pre-compute undistortion maps for fast remapping
        map1, map2 = cv2.initUndistortRectifyMap(
            camera_matrix, dist_coeffs, None, new_camera_matrix, image_size, cv2.CV_16SC2
        )

        return map1, map2

    except (json.JSONDecodeError, KeyError, ValueError) as e:
        print(f"Warning: Failed to load calibration: {e}")
        return None, None


def undistort_frame(frame: np.ndarray, map1: np.ndarray, map2: np.ndarray) -> np.ndarray:
    """
    Apply lens undistortion to a frame using pre-computed maps.

    Args:
        frame: Input image
        map1: First undistortion map from cv2.initUndistortRectifyMap
        map2: Second undistortion map

    Returns:
        Undistorted frame
    """
    return cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)


# =============================================================================
# DETECTION FUNCTIONS
# =============================================================================

def detect_markers(frame: np.ndarray, kernel: np.ndarray) -> tuple:
    """
    Detect green markers in the frame and return their centroids and mask.

    Args:
        frame: RGB image from camera
        kernel: Morphological kernel for mask cleaning

    Returns:
        Tuple of (centroids list, mask array)
    """
    # Convert to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

    # Create mask for green color
    mask = cv2.inRange(hsv, HSV_LOWER_GREEN, HSV_UPPER_GREEN)

    # Clean mask with morphological operations
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Extract centroids from valid contours
    centroids = []
    for cnt in contours:
        if cv2.contourArea(cnt) > MIN_CONTOUR_AREA:
            M = cv2.moments(cnt)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                centroids.append((cx, cy))

    return centroids, mask


def calculate_angle(centroids: list) -> float | None:
    """
    Calculate angle from vertical given two marker centroids.

    The angle is measured as deflection from straight down (vertical).
    - 0° = pole hanging straight down (no wind)
    - 90° = pole horizontal (extreme wind)

    Args:
        centroids: List of (x, y) tuples (must have exactly 2)

    Returns:
        Angle from vertical in radians [0, π/2], or None if not exactly 2 markers
    """
    if len(centroids) != 2:
        return None

    p1, p2 = centroids[0], centroids[1]

    # Ensure p1 is the top marker (smaller y in image coordinates)
    if p1[1] > p2[1]:
        p1, p2 = p2, p1

    # Vector from top marker to bottom marker
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]

    # Angle from vertical: use atan2 with (dx, dy) to get angle from +y axis
    # In image coordinates, +y is down, so a vertical pole has dx=0, dy>0
    # atan2(dx, dy) gives angle from the +y axis (vertical down)
    angle_from_vertical = abs(np.arctan2(dx, dy))

    return angle_from_vertical


def calculate_wind_speed(theta: float, theta_dot: float, params: WindParams) -> float:
    """
    Calculate wind speed from angle and angular velocity.

    Formula: v_wind = α * √tan(θ) + β * θ̇ + γ

    Args:
        theta: Angle from vertical (radians)
        theta_dot: Angular velocity (radians/second)
        params: WindParams with α, β, γ coefficients

    Returns:
        Estimated wind speed (m/s)
    """
    return params.calculate_wind_speed(theta, theta_dot)


def calculate_uncertainty_bounds(theta_history: list, theta_dot_history: list,
                                  params: WindParams, wind_speed: float) -> tuple:
    """
    Calculate asymmetric confidence bounds for wind speed.

    Instead of linear error propagation (which assumes symmetry), we directly
    evaluate the non-linear function at the bounds of the input distributions
    to get proper asymmetric confidence intervals.

    Args:
        theta_history: Rolling window of recent θ values
        theta_dot_history: Rolling window of recent θ̇ values
        params: WindParams with α, β, γ coefficients
        wind_speed: Current wind speed estimate (for fallback)

    Returns:
        Tuple of (lower_bound, upper_bound) in m/s
    """
    if len(theta_history) < 2:
        return wind_speed, wind_speed

    # Calculate statistics from measurement history
    mean_theta = np.mean(theta_history)
    mean_theta_dot = np.mean(theta_dot_history)
    sigma_theta = np.std(theta_history)
    sigma_theta_dot = np.std(theta_dot_history)

    # Evaluate function at corners of the 1-sigma box
    # For √tan(θ), lower θ gives lower v (when α > 0)
    # For θ̇ term, depends on sign of β
    theta_low = mean_theta - sigma_theta
    theta_high = mean_theta + sigma_theta
    theta_dot_low = mean_theta_dot - sigma_theta_dot
    theta_dot_high = mean_theta_dot + sigma_theta_dot

    # Evaluate at all 4 corners to find true min/max
    corners = [
        params.calculate_wind_speed(theta_low, theta_dot_low),
        params.calculate_wind_speed(theta_low, theta_dot_high),
        params.calculate_wind_speed(theta_high, theta_dot_low),
        params.calculate_wind_speed(theta_high, theta_dot_high),
    ]

    lower_bound = min(corners)
    upper_bound = max(corners)

    return lower_bound, upper_bound


# =============================================================================
# KEYBOARD INPUT
# =============================================================================

class NonBlockingInput:
    """Non-blocking keyboard input using termios."""

    def __init__(self):
        self._old_settings = None
        self._fd = sys.stdin.fileno()

    def __enter__(self):
        self._old_settings = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)
        return self

    def __exit__(self, *args):
        if self._old_settings:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_settings)

    def get_key(self) -> str | None:
        """
        Check for keypress without blocking.

        Returns:
            Single character if key pressed, None otherwise
        """
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
        return None


# =============================================================================
# VISUALIZATION FUNCTIONS
# =============================================================================

def draw_markers(frame: np.ndarray, centroids: list) -> None:
    """
    Draw detected markers on the frame (in-place).

    Args:
        frame: RGB image to draw on
        centroids: List of (x, y) centroid tuples
    """
    for cx, cy in centroids:
        cv2.circle(frame, (cx, cy), 10, (0, 255, 0), -1)  # Green filled circle
        cv2.circle(frame, (cx, cy), 12, (255, 255, 255), 2)  # White border


def draw_angle_line(frame: np.ndarray, centroids: list) -> None:
    """
    Draw line between two markers (in-place).

    Args:
        frame: RGB image to draw on
        centroids: List of exactly 2 (x, y) tuples
    """
    if len(centroids) == 2:
        cv2.line(frame, centroids[0], centroids[1], (0, 255, 0), 3)


def draw_overlay_text(frame: np.ndarray, theta: float, theta_dot: float,
                      wind_speed: float, v_lower: float, v_upper: float,
                      num_markers: int, frame_count: int) -> None:
    """
    Draw status overlay text on the frame (in-place).

    Args:
        frame: RGB image to draw on
        theta: Current angle (radians)
        theta_dot: Current angular velocity (rad/s)
        wind_speed: Current wind speed estimate (m/s)
        v_lower: Lower confidence bound (m/s)
        v_upper: Upper confidence bound (m/s)
        num_markers: Number of detected markers
        frame_count: Current frame number
    """
    theta_deg = np.degrees(theta)

    # Status color: green if tracking, red otherwise
    status_color = (0, 255, 0) if num_markers == 2 else (255, 0, 0)

    # Top-left status
    status_text = f"Frame: {frame_count} | Markers: {num_markers}/2"
    cv2.putText(frame, status_text, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, status_color, 2)

    # Angle and wind speed
    angle_text = f"Angle: {theta_deg:.1f} deg ({theta:.3f} rad)"
    cv2.putText(frame, angle_text, (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

    wind_text = f"Wind Speed: {wind_speed:.2f} [{v_lower:.2f}, {v_upper:.2f}] m/s"
    cv2.putText(frame, wind_text, (10, 90),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)


def create_plot_frame(time_history: list, wind_history: list,
                      lower_history: list, upper_history: list,
                      current_wind: float, v_lower: float, v_upper: float, fig, ax,
                      gt_time_history: list = None, gt_history: list = None) -> np.ndarray:
    """
    Create a plot image showing wind speed over time with confidence bounds.

    Args:
        time_history: List of timestamps (seconds)
        wind_history: List of wind speed values (m/s)
        lower_history: List of lower bound values (m/s)
        upper_history: List of upper bound values (m/s)
        current_wind: Current wind speed for highlighting
        v_lower: Current lower bound
        v_upper: Current upper bound
        fig: Matplotlib figure object
        ax: Matplotlib axes object
        gt_time_history: List of timestamps for ground truth (seconds)
        gt_history: List of ground truth wind speed values (m/s)

    Returns:
        RGB image of the plot (1280 x PLOT_HEIGHT)
    """
    ax.clear()

    if len(wind_history) > 0:
        t = np.array(time_history)
        w = np.array(wind_history)
        lo = np.array(lower_history)
        hi = np.array(upper_history)

        # Plot confidence band (asymmetric bounds)
        ax.fill_between(t, lo, hi, alpha=0.3, color='blue', label='Bounds')

        # Plot estimated wind speed line
        ax.plot(t, w, 'b-', linewidth=2, label='Estimate')

        # Plot ground truth if available
        if gt_time_history and gt_history and len(gt_history) > 0:
            gt_t = np.array(gt_time_history)
            gt_w = np.array(gt_history)
            ax.plot(gt_t, gt_w, 'g-', linewidth=2, label='Ground Truth')
            # Highlight current GT value
            ax.plot(gt_t[-1], gt_w[-1], 'go', markersize=8)

        # Highlight current estimate value
        if len(time_history) > 0:
            ax.plot(time_history[-1], current_wind, 'bo', markersize=8)

        ax.set_xlabel('Time (seconds)', fontsize=10)
        ax.set_ylabel('Wind Speed (m/s)', fontsize=10)
        ax.set_title('Wind Speed Over Time', fontsize=12, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='upper right', fontsize=9)

        # Show last N seconds
        ax.set_xlim(max(0, time_history[-1] - PLOT_HISTORY_SECONDS),
                    time_history[-1] + 0.5)

        # Current value annotation
        annotation = f'Estimate: {current_wind:.2f} [{v_lower:.2f}, {v_upper:.2f}] m/s'
        if gt_history and len(gt_history) > 0:
            annotation += f'\nGT: {gt_history[-1]:.2f} m/s'
        ax.text(0.02, 0.98, annotation,
                transform=ax.transAxes, fontsize=11, fontweight='bold',
                verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    else:
        ax.text(0.5, 0.5, 'Waiting for data...',
                horizontalalignment='center', verticalalignment='center',
                transform=ax.transAxes, fontsize=14)
        ax.set_xlabel('Time (seconds)')
        ax.set_ylabel('Wind Speed (m/s)')

    fig.tight_layout()

    # Render to image
    canvas = FigureCanvasAgg(fig)
    canvas.draw()
    plot_image = np.frombuffer(canvas.buffer_rgba(), dtype=np.uint8)
    plot_image = plot_image.reshape(canvas.get_width_height()[::-1] + (4,))
    plot_image = cv2.cvtColor(plot_image, cv2.COLOR_RGBA2RGB)
    plot_image = cv2.resize(plot_image, (CAMERA_RESOLUTION[0], PLOT_HEIGHT))

    return plot_image


# =============================================================================
# TERMINAL VISUALIZATION FUNCTIONS
# =============================================================================

def rgb_to_ansi(r: int, g: int, b: int) -> str:
    """Convert RGB to ANSI 256-color escape code."""
    # Use 24-bit true color for best results
    return f"\033[48;2;{r};{g};{b}m"

def frame_to_ascii(frame: np.ndarray, width: int, height: int, centroids: list = None) -> list:
    """
    Convert a frame to colored ASCII art using background colors (ASCII-safe).

    Args:
        frame: RGB image
        width: Output width in characters
        height: Output height in characters
        centroids: Optional list of marker positions to highlight

    Returns:
        List of strings (lines) with ANSI color codes for terminal display
    """
    # Apply slight blur to reduce noise before downscaling
    blurred = cv2.GaussianBlur(frame, (5, 5), 0)

    # Resize frame to target dimensions
    small = cv2.resize(blurred, (width, height), interpolation=cv2.INTER_AREA)

    # Create marker mask for highlighting
    marker_mask = np.zeros((height, width), dtype=bool)
    if centroids:
        scale_x = width / frame.shape[1]
        scale_y = height / frame.shape[0]
        for cx, cy in centroids:
            mx, my = int(cx * scale_x), int(cy * scale_y)
            # Mark area around centroid
            for dy in range(-1, 2):
                for dx in range(-1, 2):
                    ny, nx = my + dy, mx + dx
                    if 0 <= ny < height and 0 <= nx < width:
                        marker_mask[ny, nx] = True

    lines = []
    # Use background color with space character (ASCII-safe, like calibrate.py)
    for y in range(height):
        line = ""
        for x in range(width):
            r, g, b = small[y, x]

            if marker_mask[y, x]:
                # Highlight markers in bright magenta
                line += "\033[48;2;255;0;255m \033[0m"
            else:
                # Normal pixel - use background color
                line += f"\033[48;2;{r};{g};{b}m \033[0m"
        lines.append(line)

    return lines


def mask_to_ascii(frame: np.ndarray, mask: np.ndarray, width: int, height: int, centroids: list = None) -> list:
    """
    Convert a frame to ASCII with green mask overlay highlighted (ASCII-safe).

    Args:
        frame: RGB image
        mask: Binary mask (green detection)
        width: Output width in characters
        height: Output height in characters
        centroids: Optional list of marker positions

    Returns:
        List of strings (lines) with ANSI color codes
    """
    # Apply slight blur to reduce noise before downscaling
    blurred = cv2.GaussianBlur(frame, (5, 5), 0)

    # Resize both frame and mask
    small = cv2.resize(blurred, (width, height), interpolation=cv2.INTER_AREA)
    small_mask = cv2.resize(mask, (width, height), interpolation=cv2.INTER_NEAREST)

    # Create marker mask for highlighting
    marker_mask = np.zeros((height, width), dtype=bool)
    if centroids:
        scale_x = width / frame.shape[1]
        scale_y = height / frame.shape[0]
        for cx, cy in centroids:
            mx, my = int(cx * scale_x), int(cy * scale_y)
            # Mark area around centroid
            for dy in range(-1, 2):
                for dx in range(-1, 2):
                    ny, nx = my + dy, mx + dx
                    if 0 <= ny < height and 0 <= nx < width:
                        marker_mask[ny, nx] = True

    lines = []
    # Use background color with space character (ASCII-safe)
    for y in range(height):
        line = ""
        for x in range(width):
            r, g, b = small[y, x]
            is_mask = small_mask[y, x] > 0

            if marker_mask[y, x]:
                # Bright magenta for detected marker centroid
                line += "\033[48;2;255;0;255m \033[0m"
            elif is_mask:
                # Bright green for mask areas
                line += "\033[48;2;0;255;0m \033[0m"
            else:
                # Normal pixel (dimmed for contrast)
                dim = 0.4
                dr, dg, db = int(r * dim), int(g * dim), int(b * dim)
                line += f"\033[48;2;{dr};{dg};{db}m \033[0m"
        lines.append(line)

    return lines

def draw_angle_gauge(angle_deg: float, width: int = 20) -> str:
    """
    Draw a simple ASCII angle gauge.

    Args:
        angle_deg: Angle in degrees from vertical
        width: Width of the gauge

    Returns:
        Multi-line string showing angle visualization
    """
    lines = []

    # Simple pendulum visualization
    #      |      (vertical reference)
    #      |\     (pole at angle)
    #      | \
    #      |  ●

    # Clamp angle for display
    display_angle = max(-45, min(45, angle_deg))

    # Calculate pendulum end position (simplified)
    # 5 rows tall, angle affects x offset
    pendulum_len = 4
    x_offset = int(pendulum_len * np.sin(np.radians(display_angle)))

    lines.append("     │     ")  # Top pivot
    for i in range(1, pendulum_len):
        x_pos = int(i * x_offset / pendulum_len)
        spaces_left = 5 + x_pos
        if spaces_left < 0:
            spaces_left = 0
        line = " " * spaces_left + "╲" if x_offset > 0 else " " * spaces_left + "│" if x_offset == 0 else " " * spaces_left + "╱"
        lines.append(line)

    # Bottom with angle indicator
    x_pos = x_offset
    spaces = 5 + x_pos
    if spaces < 0:
        spaces = 0
    lines.append(" " * spaces + "●")
    lines.append(f"   {angle_deg:+.1f}°")

    return "\n".join(lines)

def draw_wind_bar(wind_speed: float, max_wind: float = 5.0, width: int = 30) -> str:
    """
    Draw a horizontal bar showing wind speed.

    Args:
        wind_speed: Current wind speed in m/s
        max_wind: Maximum wind speed for scale
        width: Width of the bar in characters

    Returns:
        String with colored wind speed bar
    """
    # Calculate fill amount
    fill = int((wind_speed / max_wind) * width)
    fill = max(0, min(width, fill))

    # Color gradient: green -> yellow -> red
    if wind_speed < max_wind * 0.3:
        color = "\033[92m"  # Green
    elif wind_speed < max_wind * 0.6:
        color = "\033[93m"  # Yellow
    else:
        color = "\033[91m"  # Red

    bar = color + "#" * fill + "\033[90m" + "-" * (width - fill) + "\033[0m"
    return bar


def draw_ascii_plot(wind_history: list, time_history: list, width: int = 80, height: int = 10,
                    gt_history: list = None, gt_time_history: list = None) -> list:
    """
    Draw an ASCII line plot of wind speed over time.

    Args:
        wind_history: List of wind speed values (estimate)
        time_history: List of corresponding timestamps
        width: Plot width in characters
        height: Plot height in characters
        gt_history: List of ground truth wind speed values
        gt_time_history: List of ground truth timestamps

    Returns:
        List of strings (lines) for the plot
    """
    lines = []

    if len(wind_history) < 2:
        # Not enough data - show placeholder
        lines.append(f"\033[93m{'-' * width}\033[0m")
        lines.append(f"\033[93m Wind Speed Plot \033[0m")
        lines.append(f"\033[93m{'-' * width}\033[0m")
        for _ in range(height):
            lines.append("\033[90m|" + " " * (width - 2) + "|\033[0m")
        lines.append(f"\033[90m+{'-' * (width - 2)}+\033[0m")
        lines.append("  \033[90mWaiting for data...\033[0m")
        return lines

    # Calculate plot bounds - include GT in bounds calculation
    all_winds = list(wind_history)
    if gt_history:
        all_winds.extend(gt_history)

    min_wind = max(0, min(all_winds) - 0.5)
    max_wind = max(all_winds) + 0.5
    if max_wind - min_wind < 1.0:
        max_wind = min_wind + 1.0

    min_time = min(time_history)
    max_time = max(time_history)
    if max_time - min_time < 1.0:
        max_time = min_time + 1.0

    # Create plot grid
    plot_width = width - 8  # Leave space for y-axis labels
    plot_height = height

    # Initialize grid with spaces (store tuples: (char, color_code))
    grid = [[(' ', '90') for _ in range(plot_width)] for _ in range(plot_height)]

    # Helper function to plot a series
    def plot_series(hist, t_hist, dot_char, line_char, current_char, color):
        if not hist or len(hist) < 1:
            return
        for i in range(len(hist)):
            if i >= len(t_hist):
                break
            # Map to grid coordinates
            x = int((t_hist[i] - min_time) / (max_time - min_time) * (plot_width - 1))
            y = int((hist[i] - min_wind) / (max_wind - min_wind) * (plot_height - 1))

            # Clamp to bounds
            x = max(0, min(plot_width - 1, x))
            y = max(0, min(plot_height - 1, y))

            # Invert y (0 at bottom)
            y = plot_height - 1 - y

            # Use different characters for the line
            if i == len(hist) - 1:
                grid[y][x] = (current_char, color)  # Current point
            elif grid[y][x][0] == ' ':
                grid[y][x] = (dot_char, color)  # History points

        # Connect points with line characters
        for i in range(len(hist) - 1):
            if i + 1 >= len(t_hist):
                break
            x1 = int((t_hist[i] - min_time) / (max_time - min_time) * (plot_width - 1))
            x2 = int((t_hist[i+1] - min_time) / (max_time - min_time) * (plot_width - 1))
            y1 = int((hist[i] - min_wind) / (max_wind - min_wind) * (plot_height - 1))
            y2 = int((hist[i+1] - min_wind) / (max_wind - min_wind) * (plot_height - 1))

            x1, x2 = max(0, min(plot_width-1, x1)), max(0, min(plot_width-1, x2))
            y1, y2 = max(0, min(plot_height-1, y1)), max(0, min(plot_height-1, y2))
            y1, y2 = plot_height - 1 - y1, plot_height - 1 - y2

            # Fill horizontal gaps
            if x2 > x1 + 1:
                for x in range(x1 + 1, x2):
                    t = (x - x1) / (x2 - x1)
                    y = int(y1 + t * (y2 - y1))
                    if 0 <= y < plot_height and grid[y][x][0] == ' ':
                        grid[y][x] = (line_char, color)

    # Plot ground truth first (so estimate draws on top)
    if gt_history and gt_time_history and len(gt_history) > 0:
        plot_series(gt_history, gt_time_history, '.', '-', 'G', '92')  # Green

    # Plot estimate
    plot_series(wind_history, time_history, '.', '-', 'E', '96')  # Cyan

    # Build output lines
    lines.append(f"\033[93m{'-' * width}\033[0m")
    title = "Wind Speed: \033[96mE=Estimate\033[0m  \033[92mG=Ground Truth\033[0m"
    title_pad = (width - 36) // 2  # Approximate visible length
    lines.append(f"{' ' * title_pad}{title}")
    lines.append(f"\033[93m{'-' * width}\033[0m")

    # Add y-axis labels and grid
    for row_idx, row in enumerate(grid):
        # Calculate y value for this row
        y_val = max_wind - (row_idx / (plot_height - 1)) * (max_wind - min_wind)
        y_label = f"{y_val:5.2f}"

        # Color the plot line
        row_str = ""
        for char, color in row:
            row_str += f"\033[{color}m{char}\033[0m"

        lines.append(f"\033[90m{y_label} |\033[0m{row_str}\033[90m|\033[0m")

    # X-axis
    lines.append(f"\033[90m      +{'-' * plot_width}+\033[0m")

    # X-axis labels
    x_label_left = f"{min_time:.1f}s"
    x_label_right = f"{max_time:.1f}s"
    x_label_pad = plot_width - len(x_label_left) - len(x_label_right)
    lines.append(f"\033[90m       {x_label_left}{' ' * x_label_pad}{x_label_right}\033[0m")

    return lines

def render_terminal_display(raw_frame: np.ndarray, calibrated_frame: np.ndarray,
                           mask: np.ndarray, centroids: list,
                           theta: float, wind_speed: float,
                           v_lower: float, v_upper: float,
                           elapsed: float, frame_count: int,
                           wind_history: list, time_history: list,
                           lower_history: list, upper_history: list,
                           ground_truth: float | None = None,
                           gt_history: list = None, gt_time_history: list = None,
                           tuning_state: TuningState = TuningState.IDLE,
                           tuning_samples: int = 0,
                           params: WindParams = None) -> str:
    """
    Render the complete terminal display with side-by-side camera and mask views.

    Args:
        raw_frame: Raw camera frame without calibration (RGB)
        calibrated_frame: Calibrated frame with lens distortion corrected (RGB)
        mask: Green detection mask (computed from calibrated frame)
        centroids: Detected marker positions (from calibrated frame)
        theta: Current angle in radians
        wind_speed: Current wind speed estimate
        v_lower: Lower confidence bound
        v_upper: Upper confidence bound
        elapsed: Elapsed time in seconds
        frame_count: Current frame count
        wind_history: List of recent wind speeds for stats
        time_history: List of timestamps for plot
        lower_history: List of lower bounds for plot
        upper_history: List of upper bounds for plot
        ground_truth: Ground truth wind speed from sensor (None if unavailable)
        gt_history: List of ground truth wind speed values for plotting
        gt_time_history: List of ground truth timestamps for plotting
        tuning_state: Current tuning state machine state
        tuning_samples: Number of tuning samples collected
        params: Current WindParams (for display)

    Returns:
        Complete terminal display string
    """
    lines = []

    # Get terminal size
    term_width = shutil.get_terminal_size().columns

    # Calculate preview widths for side-by-side (with 3 char gap between)
    total_preview_width = min(TERMINAL_PREVIEW_WIDTH * 2 + 3, term_width - 4)
    single_width = (total_preview_width - 3) // 2

    # Generate both ASCII views
    # Camera Preview: raw frame, no markers (shows what camera actually sees)
    raw_lines = frame_to_ascii(raw_frame, single_width, TERMINAL_PREVIEW_HEIGHT, None)
    # Green Detection: calibrated frame with mask overlay and markers
    mask_lines = mask_to_ascii(calibrated_frame, mask, single_width, TERMINAL_PREVIEW_HEIGHT, centroids)

    # Reset terminal state and move cursor to top
    # \033[0m = reset all attributes (prevents color bleed)
    # \033[H = move cursor to home position
    lines.append("\033[0m\033[H")

    # Title row
    left_title = "Camera Preview"
    right_title = "Green Detection"
    left_padding = (single_width - len(left_title)) // 2
    right_padding = (single_width - len(right_title)) // 2

    lines.append(f"\033[96m{'-' * single_width}\033[0m   \033[92m{'-' * single_width}\033[0m")
    lines.append(f"\033[96m{' ' * left_padding}{left_title}{' ' * (single_width - left_padding - len(left_title))}\033[0m   \033[92m{' ' * right_padding}{right_title}{' ' * (single_width - right_padding - len(right_title))}\033[0m")
    lines.append(f"\033[96m{'-' * single_width}\033[0m   \033[92m{'-' * single_width}\033[0m")

    # Side-by-side preview
    for i in range(len(raw_lines)):
        lines.append(f"{raw_lines[i]}   {mask_lines[i]}")

    lines.append(f"\033[96m{'-' * single_width}\033[0m   \033[92m{'-' * single_width}\033[0m")

    # Stats section
    theta_deg = np.degrees(theta)
    markers_status = f"\033[92m[*]\033[0m {len(centroids)}/2" if len(centroids) == 2 else f"\033[91m[X]\033[0m {len(centroids)}/2"

    lines.append("")
    lines.append(f"  \033[93mAngle:\033[0m {theta_deg:+6.1f} deg ({theta:.3f} rad)    \033[93mMarkers:\033[0m {markers_status}")
    lines.append("")

    # Wind speed bar - our estimate with bounds
    lines.append(f"  \033[93mWind Speed:\033[0m {draw_wind_bar(wind_speed)} {wind_speed:.2f} m/s [{v_lower:.2f}, {v_upper:.2f}]")

    # Ground truth display
    if ground_truth is not None:
        gt_bar = draw_wind_bar(ground_truth)
        error = wind_speed - ground_truth
        error_str = f"({error:+.2f})" if abs(error) < 10 else ""
        lines.append(f"  \033[95mGround Truth:\033[0m {gt_bar} {ground_truth:.2f} m/s {error_str}")
    else:
        lines.append(f"  \033[90mGround Truth: N/A (not connected)\033[0m")

    # Statistics
    if wind_history and len(wind_history) > 1:
        avg_wind = np.mean(wind_history)
        min_wind = np.min(wind_history)
        max_wind = np.max(wind_history)
        lines.append(f"  \033[90mMin: {min_wind:.2f}  Max: {max_wind:.2f}  Avg: {avg_wind:.2f} m/s\033[0m")

    lines.append("")

    # Parameters display
    if params:
        lines.append(f"  \033[90mParams: a={params.alpha:.3f}  b={params.beta:.3f}  g={params.gamma:.3f}\033[0m")

    # Tuning status
    if tuning_state == TuningState.RECORDING:
        lines.append(f"  \033[91m[REC] RECORDING TUNING DATA\033[0m ({tuning_samples} samples) - Press 't' to stop")
    elif tuning_state == TuningState.OPTIMIZING:
        lines.append(f"  \033[93m[...] OPTIMIZING...\033[0m Please wait")
    else:
        lines.append(f"  \033[90mPress 't' to start tuning data collection\033[0m")

    lines.append("")

    # ASCII Wind Speed Plot
    plot_lines = draw_ascii_plot(wind_history, time_history, width=total_preview_width, height=8,
                                 gt_history=gt_history, gt_time_history=gt_time_history)
    lines.extend(plot_lines)

    lines.append("")
    lines.append(f"  \033[96mTime:\033[0m {elapsed:.1f}s  \033[96mFrames:\033[0m {frame_count}  \033[90mCtrl+C to stop | 't' for tuning\033[0m")
    lines.append("")

    # Add reset and clear-to-end-of-line to each line to prevent artifacts
    # \033[0m resets attributes, \033[K clears to end of line
    lines = [line + "\033[0m\033[K" for line in lines]

    # Add final reset to ensure clean state
    return "\n".join(lines) + "\033[0m"


# =============================================================================
# MAIN
# =============================================================================

def main():
    """Main function: camera loop with recording, serial I/O, and tuning."""

    print("=" * 60)
    print("Wind Speed Estimator")
    print("=" * 60)

    # Load wind speed parameters
    print("Loading wind speed parameters...")
    params = load_parameters()
    print(f"  α={params.alpha:.3f}, β={params.beta:.3f}, γ={params.gamma:.3f}")
    print(f"  Formula: v_wind = {params.alpha:.3f} * θ + {params.beta:.3f} * θ̇ + {params.gamma:.3f}")
    print()

    # Load serial configuration
    print("Loading serial configuration...")
    serial_config = load_serial_config()
    serial_input = None
    serial_output = None

    if "ground_truth" in serial_config:
        gt_cfg = serial_config["ground_truth"]
        print(f"  Ground truth: {gt_cfg['name']} @ {gt_cfg['port']}")
        serial_input = SerialInput(gt_cfg["port"], gt_cfg["baud"])
        if not serial_input.start():
            print("  Warning: Could not connect to ground truth sensor")
            serial_input = None

    if "output" in serial_config:
        out_cfg = serial_config["output"]
        print(f"  Output: {out_cfg['name']} @ {out_cfg['port']}")
        serial_output = SerialOutput(out_cfg["port"], out_cfg["baud"])
        if not serial_output.start():
            print("  Warning: Could not connect to output port")
            serial_output = None

    if not serial_config:
        print("  No serial config found - running without serial I/O")
    print()

    print(f"Camera: {CAMERA_RESOLUTION[0]}x{CAMERA_RESOLUTION[1]} @ {CAMERA_FPS}fps")
    print("Controls: 't' = toggle tuning, Ctrl+C = stop")
    print("=" * 60)
    print()

    # Load camera calibration
    print("Loading camera calibration...")
    undistort_map1, undistort_map2 = load_calibration()
    if undistort_map1 is not None:
        print("  Calibration loaded - lens distortion correction enabled")
    else:
        print("  No calibration found - running without distortion correction")
        print("  (Run calibration/calibrate.py to create calibration)")
    print()

    # Create output directory
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = f"output/{timestamp}"
    os.makedirs(output_dir, exist_ok=True)

    csv_path = os.path.join(output_dir, "data.csv")
    video_path = os.path.join(output_dir, "video.mp4")
    raw_video_path = os.path.join(output_dir, "raw.mp4")
    calibrated_video_path = os.path.join(output_dir, "calibrated.mp4")
    ascii_video_path = os.path.join(output_dir, "ascii_video.txt")

    print(f"Output directory: {output_dir}")
    print(f"  - CSV: {csv_path}")
    print(f"  - Video: {video_path}")
    print(f"  - Raw video: {raw_video_path}")
    print(f"  - Calibrated video: {calibrated_video_path}")
    print(f"  - ASCII video: {ascii_video_path}")
    print()

    # Initialize camera
    print("Initializing camera...")
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"format": "RGB888", "size": CAMERA_RESOLUTION}
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(0.5)
    print("Camera ready!")
    print()

    # Initialize video writers
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    frame_size_with_plot = (CAMERA_RESOLUTION[0], CAMERA_RESOLUTION[1] + PLOT_HEIGHT)

    video_writer = cv2.VideoWriter(video_path, fourcc, CAMERA_FPS, frame_size_with_plot)
    raw_video_writer = cv2.VideoWriter(raw_video_path, fourcc, CAMERA_FPS, CAMERA_RESOLUTION)
    calibrated_video_writer = cv2.VideoWriter(calibrated_video_path, fourcc, CAMERA_FPS, CAMERA_RESOLUTION)

    # Initialize CSV file
    csv_file = open(csv_path, 'w')
    csv_file.write("timestamp,theta,theta_dot,wind_speed,v_lower,v_upper,ground_truth\n")

    # Initialize ASCII video file
    ascii_file = open(ascii_video_path, 'w')

    # Initialize matplotlib figure for plot
    fig, ax = plt.subplots(figsize=(12.8, PLOT_HEIGHT / 100))
    fig.patch.set_facecolor('white')

    # Processing state
    kernel = np.ones((5, 5), np.uint8)
    prev_theta = 0.0
    prev_time = time.time()

    # History for plotting
    time_history = []
    wind_history = []
    lower_history = []
    upper_history = []
    gt_time_history = []
    gt_history = []

    # Rolling window for uncertainty estimation
    theta_window = []
    theta_dot_window = []

    frame_count = 0
    start_time = time.time()

    # Video timing - track actual time for real-time sync
    last_video_time = 0.0
    frame_duration = 1.0 / CAMERA_FPS

    # Tuning state
    tuning_state = TuningState.IDLE
    tuning_data: list[TuningDataPoint] = []
    tuning_start_time = 0.0
    has_ground_truth_data = False

    print("Recording started...")
    print()

    # Clear screen once at start
    sys.stdout.write("\033[2J\033[H")
    sys.stdout.flush()

    with NonBlockingInput() as kb:
        try:
            while True:
                loop_start = time.time()

                # Check for keyboard input
                key = kb.get_key()
                if key == 't':
                    if tuning_state == TuningState.IDLE:
                        # Start recording tuning data
                        tuning_state = TuningState.RECORDING
                        tuning_data = []
                        tuning_start_time = time.time()
                        has_ground_truth_data = False

                        # Reset parameters to defaults if GT is connected
                        # This prevents bad previous tuning from affecting new tuning
                        if serial_input and serial_input.is_connected:
                            params = WindParams()  # Reset to defaults
                            save_parameters(params)
                    elif tuning_state == TuningState.RECORDING:
                        # Stop recording and process
                        tuning_state = TuningState.OPTIMIZING

                        # Save tuning data
                        save_tuning_data(tuning_data, has_ground_truth_data)

                        # Run optimization if we have ground truth
                        if has_ground_truth_data:
                            new_params = optimize_parameters(tuning_data)
                            if new_params:
                                params = new_params
                                save_parameters(params)

                        tuning_state = TuningState.IDLE

                # Capture frame
                # Note: Picamera2 with RGB888 format may still output BGR on some systems
                # Convert to ensure consistent RGB format for processing and display
                frame = picam2.capture_array()
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                raw_frame = frame.copy()

                # Apply lens distortion correction if calibration is available
                if undistort_map1 is not None:
                    frame = undistort_frame(frame, undistort_map1, undistort_map2)

                # Detect markers
                centroids, mask = detect_markers(frame, kernel)

                # Calculate angle
                detected_angle = calculate_angle(centroids)

                # Handle missing detection: use previous value
                if detected_angle is not None:
                    theta = detected_angle
                else:
                    theta = prev_theta

                # Calculate theta_dot
                current_time = time.time()
                dt = current_time - prev_time

                if dt > 0 and detected_angle is not None:
                    theta_dot = (theta - prev_theta) / dt
                else:
                    theta_dot = 0.0

                # Calculate wind speed using current parameters
                wind_speed = calculate_wind_speed(theta, theta_dot, params)

                # Get ground truth from serial input
                ground_truth = None
                if serial_input and serial_input.is_connected:
                    ground_truth = serial_input.get_latest()

                # Send wind speed to serial output
                if serial_output and serial_output.is_connected:
                    serial_output.send(wind_speed)

                # Collect tuning data if recording
                if tuning_state == TuningState.RECORDING:
                    elapsed_tuning = current_time - tuning_start_time
                    tuning_data.append(TuningDataPoint(
                        timestamp=elapsed_tuning,
                        theta=theta,
                        theta_dot=theta_dot,
                        ground_truth=ground_truth
                    ))
                    if ground_truth is not None:
                        has_ground_truth_data = True

                # Update rolling window for uncertainty
                theta_window.append(theta)
                theta_dot_window.append(theta_dot)
                if len(theta_window) > UNCERTAINTY_WINDOW_FRAMES:
                    theta_window.pop(0)
                    theta_dot_window.pop(0)

                # Calculate uncertainty bounds (asymmetric)
                v_lower, v_upper = calculate_uncertainty_bounds(
                    theta_window, theta_dot_window, params, wind_speed
                )

                # Update history with actual elapsed time
                elapsed = current_time - start_time
                time_history.append(elapsed)
                wind_history.append(wind_speed)
                lower_history.append(v_lower)
                upper_history.append(v_upper)

                # Keep last N seconds based on actual time
                while time_history and (elapsed - time_history[0]) > PLOT_HISTORY_SECONDS:
                    time_history.pop(0)
                    wind_history.pop(0)
                    lower_history.pop(0)
                    upper_history.pop(0)

                # Update ground truth history if available
                if ground_truth is not None:
                    gt_time_history.append(elapsed)
                    gt_history.append(ground_truth)
                    # Keep GT history in sync with time window
                    while gt_time_history and (elapsed - gt_time_history[0]) > PLOT_HISTORY_SECONDS:
                        gt_time_history.pop(0)
                        gt_history.pop(0)

                # Write to CSV
                gt_str = f"{ground_truth:.4f}" if ground_truth is not None else ""
                csv_file.write(f"{elapsed:.3f},{theta:.6f},{theta_dot:.6f},{wind_speed:.4f},{v_lower:.4f},{v_upper:.4f},{gt_str}\n")

                # Keep a clean copy of calibrated frame for terminal display
                calibrated_frame_clean = frame.copy()

                # Draw on frame (for video output)
                draw_markers(frame, centroids)
                draw_angle_line(frame, centroids)
                draw_overlay_text(frame, theta, theta_dot, wind_speed, v_lower, v_upper, len(centroids), frame_count)

                # Create plot
                plot_frame = create_plot_frame(time_history, wind_history,
                                               lower_history, upper_history,
                                               wind_speed, v_lower, v_upper, fig, ax,
                                               gt_time_history, gt_history)

                # Combine frame and plot
                combined_frame = np.vstack([frame, plot_frame])

                # Write videos - duplicate frames to maintain real-time sync
                combined_frame_bgr = cv2.cvtColor(combined_frame, cv2.COLOR_RGB2BGR)
                raw_frame_bgr = cv2.cvtColor(raw_frame, cv2.COLOR_RGB2BGR)
                calibrated_frame_bgr = cv2.cvtColor(calibrated_frame_clean, cv2.COLOR_RGB2BGR)

                while last_video_time < elapsed:
                    video_writer.write(combined_frame_bgr)
                    raw_video_writer.write(raw_frame_bgr)
                    calibrated_video_writer.write(calibrated_frame_bgr)
                    last_video_time += frame_duration

                # Update terminal display (every 3 frames to reduce CPU load)
                if frame_count % 3 == 0:
                    # Periodic full clear to prevent character degradation (every ~10 seconds)
                    if frame_count % 300 == 0:
                        sys.stdout.write("\033[2J\033[H\033[0m")

                    display = render_terminal_display(
                        raw_frame, calibrated_frame_clean, mask, centroids, theta, wind_speed,
                        v_lower, v_upper, elapsed, frame_count, wind_history, time_history,
                        lower_history, upper_history,
                        ground_truth=ground_truth,
                        gt_history=gt_history, gt_time_history=gt_time_history,
                        tuning_state=tuning_state,
                        tuning_samples=len(tuning_data),
                        params=params
                    )
                    sys.stdout.write(display)
                    sys.stdout.flush()

                    # Save ASCII frame to file (with frame separator)
                    ascii_file.write(f"=== FRAME {frame_count} | Time: {elapsed:.2f}s ===\n")
                    ascii_file.write(display)
                    ascii_file.write("\n\n")

                # Update state for next iteration
                prev_theta = theta
                prev_time = current_time
                frame_count += 1

                # Small sleep to prevent CPU overload
                time.sleep(0.001)

        except KeyboardInterrupt:
            pass

    # Clear screen and reset cursor
    sys.stdout.write("\033[2J\033[H\033[0m")
    sys.stdout.flush()
    print("Stopping...")

    # Cleanup
    csv_file.close()
    ascii_file.close()
    video_writer.release()
    raw_video_writer.release()
    calibrated_video_writer.release()
    picam2.stop()
    picam2.close()
    plt.close(fig)

    # Close serial connections
    if serial_input:
        serial_input.close()
    if serial_output:
        serial_output.close()

    actual_duration = time.time() - start_time

    print()
    print("=" * 60)
    print("Recording complete!")
    print("=" * 60)
    print(f"Total frames captured: {frame_count}")
    print(f"Actual duration: {actual_duration:.2f} seconds")
    print(f"Effective FPS: {frame_count / actual_duration:.1f}")
    print()
    print(f"Final parameters: α={params.alpha:.3f}, β={params.beta:.3f}, γ={params.gamma:.3f}")
    print()
    print(f"Files saved:")
    print(f"  - {csv_path}")
    print(f"  - {video_path}")
    print(f"  - {raw_video_path}")
    print(f"  - {calibrated_video_path}")
    print(f"  - {ascii_video_path}")

    if wind_history:
        print()
        print("Statistics:")
        print(f"  Wind Speed - Avg: {np.mean(wind_history):.2f} m/s")
        print(f"             - Min: {np.min(wind_history):.2f} m/s")
        print(f"             - Max: {np.max(wind_history):.2f} m/s")
        print(f"             - Std: {np.std(wind_history):.2f} m/s")


if __name__ == "__main__":
    main()
