#!/usr/bin/env python3
"""
Camera Calibration Script

Captures images of a checkerboard pattern and computes camera calibration
parameters (camera matrix and distortion coefficients).

Usage:
    python calibrate.py

Instructions:
    1. Hold the checkerboard in front of the camera
    2. Move it to different positions and angles
    3. The script auto-captures when a stable, valid pose is detected
    4. After enough captures (default: 7), calibration is computed and saved

Press Ctrl+C to quit early (will still save if enough frames captured).
"""

import numpy as np
import cv2
from picamera2 import Picamera2
import json
import os
import time
import sys
import shutil
import select
import termios
import tty
from datetime import datetime

# =============================================================================
# CONFIGURATION
# =============================================================================

# Checkerboard parameters (inner corners)
# For a 5x8 checkerboard with 43mm squares
CHECKERBOARD_ROWS = 5 - 1  # Inner corners = squares - 1
CHECKERBOARD_COLS = 8 - 1  # Inner corners = squares - 1
SQUARE_SIZE_MM = 43.0

# Camera settings (should match wind_speed_estimator.py)
CAMERA_RESOLUTION = (1280, 720)

# Calibration settings
MIN_CAPTURES = 7   # Minimum frames needed for distortion calibration
MAX_CAPTURES = 15  # Stop after this many
STABILITY_FRAMES = 10  # Frames checkerboard must be stable before auto-capture
STABILITY_THRESHOLD = 5.0  # Max pixel movement for "stable"
MIN_COVERAGE = 0.3  # Minimum image coverage by checkerboard
CAPTURE_DELAY = 1.0  # Minimum seconds between auto-captures

# Terminal display settings
TERMINAL_PREVIEW_WIDTH = 80  # Characters wide for camera preview
TERMINAL_PREVIEW_HEIGHT = 20  # Characters tall for camera preview

# Output path
CONFIG_PATH = os.path.join(os.path.dirname(__file__), "camera_config.json")


# =============================================================================
# CALIBRATION FUNCTIONS
# =============================================================================

def find_checkerboard(frame: np.ndarray) -> tuple:
    """
    Find checkerboard corners in a frame.

    Args:
        frame: RGB image

    Returns:
        Tuple of (success, corners, gray_image)
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

    # Find checkerboard corners
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
    ret, corners = cv2.findChessboardCorners(
        gray,
        (CHECKERBOARD_COLS, CHECKERBOARD_ROWS),
        flags
    )

    if ret:
        # Refine corner positions
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

    return ret, corners, gray


def calculate_coverage(corners: np.ndarray, image_size: tuple) -> float:
    """
    Calculate what fraction of the image the checkerboard covers.

    Args:
        corners: Detected corner positions
        image_size: (width, height) of image

    Returns:
        Coverage ratio (0 to 1)
    """
    if corners is None:
        return 0.0

    # Get bounding box of corners
    x_coords = corners[:, 0, 0]
    y_coords = corners[:, 0, 1]

    width = np.max(x_coords) - np.min(x_coords)
    height = np.max(y_coords) - np.min(y_coords)

    board_area = width * height
    image_area = image_size[0] * image_size[1]

    return board_area / image_area


def check_stability(current_corners: np.ndarray, history: list) -> bool:
    """
    Check if checkerboard position has been stable.

    Args:
        current_corners: Current corner positions
        history: List of previous corner positions

    Returns:
        True if stable for enough frames
    """
    if len(history) < STABILITY_FRAMES:
        return False

    # Check movement over recent frames
    for prev_corners in history[-STABILITY_FRAMES:]:
        if prev_corners is None:
            return False
        diff = np.abs(current_corners - prev_corners).max()
        if diff > STABILITY_THRESHOLD:
            return False

    return True


def check_pose_diversity(new_corners: np.ndarray, captured_corners: list) -> bool:
    """
    Check if new pose is sufficiently different from already captured poses.

    Args:
        new_corners: Corners from potential new capture
        captured_corners: List of already captured corner sets

    Returns:
        True if pose is diverse enough
    """
    if len(captured_corners) == 0:
        return True

    # Calculate center and spread of new pose
    new_center = np.mean(new_corners, axis=0)

    for prev_corners in captured_corners:
        prev_center = np.mean(prev_corners, axis=0)
        center_dist = np.linalg.norm(new_center - prev_center)

        # If too similar, reject
        if center_dist < 50:  # pixels
            return False

    return True


def compute_calibration(object_points: list, image_points: list,
                        image_size: tuple) -> tuple:
    """
    Compute camera calibration from collected points.

    Args:
        object_points: List of 3D object points
        image_points: List of 2D image points
        image_size: (width, height)

    Returns:
        Tuple of (camera_matrix, dist_coeffs, reprojection_error)
    """
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        object_points,
        image_points,
        image_size,
        None,
        None
    )

    # Calculate reprojection error
    total_error = 0
    for i in range(len(object_points)):
        projected, _ = cv2.projectPoints(
            object_points[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs
        )
        error = cv2.norm(image_points[i], projected, cv2.NORM_L2) / len(projected)
        total_error += error

    mean_error = total_error / len(object_points)

    return camera_matrix, dist_coeffs, mean_error


def save_calibration(camera_matrix: np.ndarray, dist_coeffs: np.ndarray,
                     image_size: tuple, reprojection_error: float) -> None:
    """
    Save calibration to JSON config file.

    Args:
        camera_matrix: 3x3 camera matrix
        dist_coeffs: Distortion coefficients
        image_size: (width, height)
        reprojection_error: Mean reprojection error
    """
    config = {
        "camera_matrix": camera_matrix.tolist(),
        "distortion_coefficients": dist_coeffs.flatten().tolist(),
        "image_size": list(image_size),
        "calibration_date": datetime.now().isoformat(),
        "reprojection_error": reprojection_error,
        "checkerboard_size": [CHECKERBOARD_COLS, CHECKERBOARD_ROWS],
        "square_size_mm": SQUARE_SIZE_MM
    }

    with open(CONFIG_PATH, 'w') as f:
        json.dump(config, f, indent=2)


# =============================================================================
# TERMINAL VISUALIZATION FUNCTIONS
# =============================================================================

def frame_to_ascii(frame: np.ndarray, corners: np.ndarray, width: int, height: int) -> list:
    """
    Convert a frame to colored ASCII art with checkerboard corners highlighted.

    Args:
        frame: RGB image
        corners: Detected checkerboard corners (or None)
        width: Output width in characters
        height: Output height in characters

    Returns:
        List of strings (lines) with ANSI color codes for terminal display
    """
    # Apply slight blur to reduce noise before downscaling
    blurred = cv2.GaussianBlur(frame, (5, 5), 0)

    # Resize frame to target dimensions
    small = cv2.resize(blurred, (width, height), interpolation=cv2.INTER_AREA)

    # Create corner mask for highlighting
    corner_mask = np.zeros((height, width), dtype=bool)
    if corners is not None:
        scale_x = width / frame.shape[1]
        scale_y = height / frame.shape[0]
        for corner in corners:
            cx = int(corner[0, 0] * scale_x)
            cy = int(corner[0, 1] * scale_y)
            # Mark area around corner
            for dy in range(-1, 2):
                for dx in range(-1, 2):
                    ny, nx = cy + dy, cx + dx
                    if 0 <= ny < height and 0 <= nx < width:
                        corner_mask[ny, nx] = True

    lines = []
    # Use background color with space character (ASCII-safe)
    for y in range(height):
        line = ""
        for x in range(width):
            r, g, b = small[y, x]

            if corner_mask[y, x]:
                # Highlight corners in bright yellow
                line += f"\033[48;2;255;255;0m \033[0m"
            else:
                # Normal pixel - use background color
                line += f"\033[48;2;{r};{g};{b}m \033[0m"
        lines.append(line)

    return lines


def draw_progress_bar(current: int, minimum: int, maximum: int, width: int = 40) -> str:
    """
    Draw a progress bar showing capture progress.

    Args:
        current: Current number of captures
        minimum: Minimum required captures
        maximum: Maximum captures
        width: Width of the bar in characters

    Returns:
        String with colored progress bar
    """
    # Calculate fill amounts
    min_pos = int((minimum / maximum) * width)
    current_pos = int((current / maximum) * width)
    current_pos = min(current_pos, width)

    bar = ""
    for i in range(width):
        if i < current_pos:
            if current >= minimum:
                bar += "\033[92m#\033[0m"  # Green - past minimum
            else:
                bar += "\033[93m#\033[0m"  # Yellow - not yet at minimum
        elif i == min_pos:
            bar += "\033[90m|\033[0m"  # Marker for minimum
        else:
            bar += "\033[90m-\033[0m"  # Empty

    return bar


def draw_coverage_bar(coverage: float, min_coverage: float, width: int = 20) -> str:
    """
    Draw a bar showing checkerboard coverage.

    Args:
        coverage: Current coverage ratio (0-1)
        min_coverage: Minimum required coverage
        width: Width of the bar

    Returns:
        String with colored coverage bar
    """
    fill = int(coverage * width)
    min_pos = int(min_coverage * width)

    bar = ""
    for i in range(width):
        if i < fill:
            if coverage >= min_coverage:
                bar += "\033[92m#\033[0m"  # Green - sufficient
            else:
                bar += "\033[91m#\033[0m"  # Red - insufficient
        elif i == min_pos:
            bar += "\033[90m|\033[0m"  # Marker for minimum
        else:
            bar += "\033[90m-\033[0m"  # Empty

    return bar


def render_terminal_display(frame: np.ndarray, corners: np.ndarray,
                           num_captured: int, is_stable: bool, is_diverse: bool,
                           coverage: float, message: str, found: bool) -> str:
    """
    Render the complete terminal display for calibration.

    Args:
        frame: Current camera frame (RGB)
        corners: Detected corners (or None)
        num_captured: Number of captured frames
        is_stable: Whether checkerboard is stable
        is_diverse: Whether pose is diverse from previous captures
        coverage: Image coverage ratio
        message: Status message
        found: Whether checkerboard was found

    Returns:
        Complete terminal display string
    """
    lines = []

    # Get terminal size
    term_width = shutil.get_terminal_size().columns

    # Calculate preview width
    preview_width = min(TERMINAL_PREVIEW_WIDTH, term_width - 4)

    # Generate ASCII preview
    preview_lines = frame_to_ascii(frame, corners, preview_width, TERMINAL_PREVIEW_HEIGHT)

    # Move cursor to top (no clear - reduces flashing)
    lines.append("\033[H")

    # Title
    title = "Camera Calibration"
    lines.append(f"\033[96m{'-' * preview_width}\033[0m")
    title_pad = (preview_width - len(title)) // 2
    lines.append(f"\033[96m{' ' * title_pad}{title}\033[0m")
    lines.append(f"\033[96m{'-' * preview_width}\033[0m")

    # Camera preview
    lines.extend(preview_lines)
    lines.append(f"\033[96m{'-' * preview_width}\033[0m")

    # Status section
    lines.append("")

    # Detection status (ASCII-safe icons)
    if not found:
        status_icon = "\033[91m[X]\033[0m"
        status_text = "No checkerboard detected"
    elif not is_stable:
        status_icon = "\033[93m[~]\033[0m"
        status_text = "Hold still..."
    elif not is_diverse:
        status_icon = "\033[93m[>]\033[0m"
        status_text = "Move to new position"
    elif coverage < MIN_COVERAGE:
        status_icon = "\033[93m[^]\033[0m"
        status_text = "Move closer"
    else:
        status_icon = "\033[92m[*]\033[0m"
        status_text = "Capturing!"

    lines.append(f"  {status_icon} \033[1m{status_text}\033[0m")
    lines.append("")

    # Progress bar
    lines.append(f"  \033[93mProgress:\033[0m {draw_progress_bar(num_captured, MIN_CAPTURES, MAX_CAPTURES)} {num_captured}/{MIN_CAPTURES}")

    # Coverage bar
    lines.append(f"  \033[93mCoverage:\033[0m {draw_coverage_bar(coverage, MIN_COVERAGE)} {coverage*100:.0f}%")

    lines.append("")

    # Message (pad to fixed width to avoid leftover characters)
    lines.append(f"  \033[90m{message:<60}\033[0m")

    lines.append("")

    # Instructions
    lines.append(f"  \033[90mControls: [c] Force capture  [r] Reset  [Ctrl+C] Quit & save\033[0m")
    lines.append("")

    # Checkerboard info
    lines.append(f"  \033[90mCheckerboard: {CHECKERBOARD_COLS+1}x{CHECKERBOARD_ROWS+1} squares, {SQUARE_SIZE_MM}mm\033[0m")

    # Add clear-to-end-of-line to each line to prevent artifacts
    lines = [line + "\033[K" for line in lines]

    return "\n".join(lines)


def get_key_nonblocking() -> str | None:
    """
    Get a key press without blocking.

    Returns:
        Key character or None if no key pressed
    """
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return None


# =============================================================================
# MAIN
# =============================================================================

def main():
    """Main calibration routine."""

    print("\033[2J\033[H")  # Clear screen
    print("=" * 60)
    print("Camera Calibration")
    print("=" * 60)
    print(f"Checkerboard: {CHECKERBOARD_COLS + 1} x {CHECKERBOARD_ROWS + 1} squares")
    print(f"Square size: {SQUARE_SIZE_MM} mm")
    print(f"Target captures: {MIN_CAPTURES} - {MAX_CAPTURES}")
    print(f"Output: {CONFIG_PATH}")
    print("=" * 60)
    print()
    print("Instructions:")
    print("  1. Hold checkerboard in view of camera")
    print("  2. Move to different positions and angles")
    print("  3. Hold still when prompted - auto-capture will trigger")
    print("  4. Cover different parts of the image for best results")
    print()
    print("Controls:")
    print("  [c] - Force capture current frame")
    print("  [r] - Reset and start over")
    print("  [Ctrl+C] - Quit and save (if enough captures)")
    print()
    input("Press Enter to start...")

    # Initialize camera
    print("\nInitializing camera...")
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"format": "RGB888", "size": CAMERA_RESOLUTION}
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(0.5)
    print("Camera ready!")

    # Prepare object points (same for all frames)
    objp = np.zeros((CHECKERBOARD_ROWS * CHECKERBOARD_COLS, 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHECKERBOARD_COLS, 0:CHECKERBOARD_ROWS].T.reshape(-1, 2)
    objp *= SQUARE_SIZE_MM

    # Storage
    object_points = []  # 3D points
    image_points = []   # 2D points
    captured_corners = []  # For diversity checking

    # State
    corner_history = []
    last_capture_time = 0
    message = "Move checkerboard into view"

    # Set up terminal for non-blocking input
    old_settings = termios.tcgetattr(sys.stdin)

    try:
        # Set terminal to raw mode for non-blocking input
        tty.setcbreak(sys.stdin.fileno())

        # Clear screen once at start
        sys.stdout.write("\033[2J\033[H")
        sys.stdout.flush()

        while len(object_points) < MAX_CAPTURES:
            # Capture frame
            frame = picam2.capture_array()

            # Find checkerboard
            found, corners, gray = find_checkerboard(frame)

            # Update history
            corner_history.append(corners if found else None)
            if len(corner_history) > STABILITY_FRAMES + 5:
                corner_history.pop(0)

            # Calculate metrics
            coverage = calculate_coverage(corners, CAMERA_RESOLUTION) if found else 0
            is_stable = check_stability(corners, corner_history) if found else False
            is_diverse = check_pose_diversity(corners, captured_corners) if found else False

            # Auto-capture logic
            current_time = time.time()
            time_since_capture = current_time - last_capture_time

            should_capture = (
                found and
                is_stable and
                is_diverse and
                coverage >= MIN_COVERAGE and
                time_since_capture >= CAPTURE_DELAY
            )

            if should_capture:
                object_points.append(objp.copy())
                image_points.append(corners.copy())
                captured_corners.append(corners.copy())
                last_capture_time = current_time
                message = f"Captured frame {len(object_points)}! Move to new position."
            elif found and not is_diverse:
                message = "Move to a different position/angle"
            elif found and coverage < MIN_COVERAGE:
                message = f"Move closer (coverage: {coverage*100:.0f}%, need {MIN_COVERAGE*100:.0f}%)"
            elif found and not is_stable:
                message = "Hold still..."
            elif not found:
                message = "Move checkerboard into view"

            # Render terminal display
            display = render_terminal_display(
                frame, corners, len(object_points),
                is_stable, is_diverse, coverage, message, found
            )
            sys.stdout.write(display)
            sys.stdout.flush()

            # Handle key input (non-blocking)
            key = get_key_nonblocking()

            if key == 'c' and found:
                # Force capture
                object_points.append(objp.copy())
                image_points.append(corners.copy())
                captured_corners.append(corners.copy())
                last_capture_time = current_time
                message = f"Force captured frame {len(object_points)}!"
            elif key == 'r':
                # Reset
                object_points.clear()
                image_points.clear()
                captured_corners.clear()
                corner_history.clear()
                message = "Reset - start over"

            # Small sleep to prevent CPU overload
            time.sleep(0.03)

    except KeyboardInterrupt:
        pass

    finally:
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

        # Clear screen
        sys.stdout.write("\033[2J\033[H\033[0m")
        sys.stdout.flush()

        picam2.stop()
        picam2.close()

    # Compute and save calibration if we have enough frames
    if len(object_points) >= MIN_CAPTURES:
        print()
        print("=" * 60)
        print("Computing calibration...")
        print("=" * 60)

        camera_matrix, dist_coeffs, reproj_error = compute_calibration(
            object_points, image_points, CAMERA_RESOLUTION
        )

        print(f"\nCalibration successful!")
        print(f"  Frames used: {len(object_points)}")
        print(f"  Reprojection error: {reproj_error:.4f} pixels")
        print()
        print("Camera Matrix:")
        print(camera_matrix)
        print()
        print("Distortion Coefficients:")
        print(dist_coeffs.flatten())

        save_calibration(camera_matrix, dist_coeffs, CAMERA_RESOLUTION, reproj_error)

        print(f"\nCalibration saved to: {CONFIG_PATH}")
        print()
        print("Calibration complete! You can now run wind_speed_estimator.py")
        print("The calibration will be automatically applied.")

    else:
        print()
        print("=" * 60)
        print(f"Not enough frames captured ({len(object_points)}/{MIN_CAPTURES})")
        print("Run calibration again to collect more frames.")
        print("=" * 60)


if __name__ == "__main__":
    main()
