#!/usr/bin/env python3
"""
Quick script to release the camera if it's stuck
Run this if you get "Device or resource busy" error
"""

from picamera2 import Picamera2
import sys

try:
    # Try to get any open camera instances
    picam2 = Picamera2()
    picam2.stop()
    picam2.close()
    print("Camera released successfully!")
except Exception as e:
    print(f"Note: {e}")
    print("Camera may already be released or not in use.")

sys.exit(0)
