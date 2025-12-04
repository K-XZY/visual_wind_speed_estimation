"""
Serial I/O module for wind speed estimation.

Provides:
- SerialInput: Non-blocking reader for ground truth sensor
- SerialOutput: Writer for STM32 output
- load_serial_config: Load port configuration from shared config.yaml
"""

import os
import threading
import time
from typing import Optional

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False
    print("Warning: pyserial not installed. Serial I/O disabled.")

try:
    import yaml
    YAML_AVAILABLE = True
except ImportError:
    YAML_AVAILABLE = False


# Default config path (shared with GUI)
DEFAULT_CONFIG_PATH = os.path.join(
    os.path.dirname(__file__), "GUI_Rev3", "config.yaml"
)


class SerialInput:
    """
    Non-blocking serial reader for ground truth sensor.

    Runs a background thread that continuously reads from serial port.
    Use get_latest() to retrieve the most recent value.
    """

    def __init__(self, port: str, baud: int = 115200):
        """
        Initialize serial input.

        Args:
            port: Serial port path (e.g., '/dev/ttyUSB0')
            baud: Baud rate (default 115200)
        """
        self._port = port
        self._baud = baud
        self._latest_value: Optional[float] = None
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._connected = False

    def start(self) -> bool:
        """
        Start the background reader thread.

        Returns:
            True if started successfully, False otherwise
        """
        if not SERIAL_AVAILABLE:
            print(f"Serial not available, cannot connect to {self._port}")
            return False

        try:
            self._serial = serial.Serial(self._port, self._baud, timeout=0.1)
            self._connected = True
            self._running = True
            self._thread = threading.Thread(target=self._read_loop, daemon=True)
            self._thread.start()
            print(f"SerialInput started on {self._port} @ {self._baud}")
            return True
        except Exception as e:
            print(f"Failed to open serial input {self._port}: {e}")
            self._connected = False
            return False

    def _read_loop(self) -> None:
        """Background thread that continuously reads serial data."""
        while self._running:
            try:
                line = self._serial.readline().decode(errors="ignore").strip()
                if line:
                    # Parse float value (format: just a number per line)
                    value = float(line)
                    with self._lock:
                        self._latest_value = value
            except ValueError:
                # Invalid format, skip
                continue
            except Exception as e:
                if self._running:
                    print(f"Serial read error: {e}")
                break

    def get_latest(self) -> Optional[float]:
        """
        Get the most recent value from the sensor.

        Returns:
            Latest float value, or None if no data received yet
        """
        with self._lock:
            return self._latest_value

    @property
    def is_connected(self) -> bool:
        """Check if serial port is connected."""
        return self._connected

    def close(self) -> None:
        """Stop the reader and close the serial port."""
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if hasattr(self, '_serial') and self._serial.is_open:
            self._serial.close()
        self._connected = False
        print(f"SerialInput closed on {self._port}")


class SerialOutput:
    """
    Serial writer for STM32 output.

    Runs a background thread that sends the latest wind speed value at 200Hz.
    Use send() to update the value; the thread handles transmission.
    """

    # Output rate in Hz
    OUTPUT_RATE_HZ = 200

    def __init__(self, port: str, baud: int = 115200):
        """
        Initialize serial output.

        Args:
            port: Serial port path (e.g., '/dev/ttyACM0')
            baud: Baud rate (default 115200)
        """
        self._port = port
        self._baud = baud
        self._serial: Optional[serial.Serial] = None
        self._connected = False
        self._latest_value: float = 0.0
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None

    def start(self) -> bool:
        """
        Open the serial port and start the 200Hz output thread.

        Returns:
            True if opened successfully, False otherwise
        """
        if not SERIAL_AVAILABLE:
            print(f"Serial not available, cannot connect to {self._port}")
            return False

        try:
            self._serial = serial.Serial(
                self._port,
                self._baud,
                timeout=0.1,
                write_timeout=2
            )
            self._connected = True
            self._running = True
            self._thread = threading.Thread(target=self._write_loop, daemon=True)
            self._thread.start()
            print(f"SerialOutput started on {self._port} @ {self._baud} ({self.OUTPUT_RATE_HZ}Hz)")
            return True
        except Exception as e:
            print(f"Failed to open serial output {self._port}: {e}")
            self._connected = False
            return False

    def _write_loop(self) -> None:
        """Background thread that sends data at 200Hz."""
        interval = 1.0 / self.OUTPUT_RATE_HZ
        next_send_time = time.time()

        while self._running:
            now = time.time()

            if now >= next_send_time:
                try:
                    with self._lock:
                        value = self._latest_value

                    # Format: float value followed by CRLF (matching CameraTest.py)
                    # Note: No flush() - it blocks waiting for hardware FIFO to drain,
                    # which limits throughput to ~110Hz. The OS write buffer handles it.
                    message = f"{value:.4f}\r\n"
                    self._serial.write(message.encode("utf-8"))

                    # Schedule next send
                    next_send_time += interval

                    # If we've fallen behind, reset timing
                    if next_send_time < now:
                        next_send_time = now + interval

                except Exception as e:
                    if self._running:
                        print(f"Serial write error: {e}")
                    break
            else:
                # Sleep until next send time (with small margin)
                sleep_time = next_send_time - now
                if sleep_time > 0:
                    time.sleep(min(sleep_time, interval / 2))

    def send(self, value: float) -> bool:
        """
        Update the wind speed value to be sent.

        The background thread will transmit this value at 200Hz.

        Args:
            value: Wind speed in m/s

        Returns:
            True if value was updated, False if not connected
        """
        if not self._connected:
            return False

        with self._lock:
            self._latest_value = value
        return True

    @property
    def is_connected(self) -> bool:
        """Check if serial port is connected."""
        return self._connected

    def close(self) -> None:
        """Stop the writer thread and close the serial port."""
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if self._serial and self._serial.is_open:
            # Flush remaining data before closing
            try:
                self._serial.flush()
            except Exception:
                pass
            self._serial.close()
        self._connected = False
        print(f"SerialOutput closed on {self._port}")


def load_serial_config(config_path: Optional[str] = None) -> dict:
    """
    Load serial port configuration from config.yaml.

    Args:
        config_path: Path to config.yaml (default: ../Updated_GUI_Rev3/config.yaml)

    Returns:
        Dict with 'ground_truth' and 'output' keys containing port info,
        or empty dict if config not found
    """
    if config_path is None:
        config_path = DEFAULT_CONFIG_PATH

    # Normalize path
    config_path = os.path.normpath(os.path.abspath(config_path))

    if not os.path.exists(config_path):
        print(f"Config not found: {config_path}")
        return {}

    if not YAML_AVAILABLE:
        print("PyYAML not installed, cannot load config")
        return {}

    try:
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f) or {}
    except Exception as e:
        print(f"Failed to load config: {e}")
        return {}

    devices = config.get("devices", [])
    if not devices:
        print("No devices configured in config.yaml")
        return {}

    result = {}

    # First device = ground truth (input)
    if len(devices) >= 1:
        gt = devices[0]
        result["ground_truth"] = {
            "port": gt.get("port"),
            "baud": int(gt.get("baud", 115200)),
            "name": gt.get("name", "Ground Truth")
        }

    # Second device = student/output
    if len(devices) >= 2:
        out = devices[1]
        result["output"] = {
            "port": out.get("port"),
            "baud": int(out.get("baud", 115200)),
            "name": out.get("name", "Student")
        }

    return result
