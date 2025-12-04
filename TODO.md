# TODO: GUI Integration & Serial Communication

## Current Status

The core wind speed estimation is **complete**:
- [x] Camera calibration using checkerboard pattern
- [x] Green tape detection (HSV color space)
- [x] Pole angle calculation from marker position
- [x] Real-time video processing pipeline
- [x] Angle visualization overlay
- [x] Wind speed estimation (linear model: `v = α*θ + β*θ̇ + γ`)
- [x] Serial I/O module (`serial_io.py`)
- [x] PSO tuning system (`tuning/pso.py`)
- [x] Parameter persistence (`tuning/parameters.json`)
- [x] STM32 UART bridge firmware (`UART_streaming/`)

## Next Steps: GUI Integration

### Phase 1: Test Ground Truth Reading (on Pi)
- [ ] Connect GT sensor to Pi via USB
- [ ] Identify GT sensor port (`ls /dev/tty*`)
- [ ] Update `GUI_Rev3/config.yaml` with correct GT port if needed
- [ ] Run `wind_speed_estimator.py` and verify GT values displayed
- [ ] Test tuning mode (press 't') with GT connected

### Phase 2: Test STM32 Bridge (Pi → Laptop)
- [x] Flash STM32 with UART_streaming firmware
- [ ] Wire Pi to STM32 (TX→PA10, GND→GND)
- [ ] Enable Pi hardware UART if needed (`raspi-config`)
- [ ] Test serial output: `echo "1.234" > /dev/serial0`
- [ ] Verify laptop receives data via STM32 USB

### Phase 3: Demo Day Integration
- [ ] Test full pipeline: Pi camera → angle → wind speed → STM32 → Laptop GUI
- [ ] Document complete workflow

---

## Hardware Setup

### Phase 1 - Tuning (on Pi only)
```
GT Sensor ──USB──► Pi (/dev/ttyUSB0)
                   │
                   └─► wind_speed_estimator.py
                       - Reads GT via SerialInput
                       - Press 't' to tune
                       - PSO optimizes α,β,γ
```

### Phase 2 - Marking (Pi → Laptop via STM32)
```
Pi GPIO14 (TX, Pin 8) ────► STM32 PA10 (D2/RX)
Pi GND (Pin 6) ───────────► STM32 GND
                              │
                              └──► STM32 USB ──► Laptop (/dev/ttyACM0)
                                                 │
                                                 └─► GUI_Rev3 reads wind speed

GT Sensor ────────────────────────────────────► Laptop (separate USB)
                                                 │
                                                 └─► GUI_Rev3 reads ground truth
```

---

## Wiring Reference

### Pi GPIO Pinout (relevant pins)
```
Pin 6  = GND
Pin 8  = GPIO14 = UART TX  ──► to STM32
Pin 10 = GPIO15 = UART RX  (not used)
```

### STM32 Nucleo-F411RE
```
PA10 / D2 = USART1 RX  ◄── from Pi TX
GND       = Ground     ◄── from Pi GND
USB       = to Laptop
```

---

## Configuration

### `GUI_Rev3/config.yaml` (on Pi)
```yaml
devices:
  # GT sensor input (for tuning)
  - name: "Ground Truth"
    port: "/dev/ttyUSB0"    # Update after `ls /dev/tty*`
    baud: 115200

  # Output via hardware UART → STM32 → Laptop
  - name: "Student"
    port: "/dev/serial0"    # Pi hardware UART (GPIO14 TX)
    baud: 115200
```

---

## Pi UART Setup (if needed)

If `/dev/serial0` doesn't exist or isn't working:

```bash
# Enable hardware UART
sudo raspi-config
# → Interface Options → Serial Port
# → Login shell over serial: NO
# → Serial port hardware enabled: YES
# → Reboot

# Verify UART is available
ls -la /dev/serial0
# Should show: /dev/serial0 -> ttyAMA0
```

---

## Testing Commands

```bash
# On Pi: Find connected serial devices
ls /dev/tty*

# On Pi: Test writing to UART (should appear on laptop)
echo "test123" > /dev/serial0

# On Laptop: Read from STM32 USB
cat /dev/ttyACM0

# On Pi: Run wind speed estimator
python wind_speed_estimator.py
```
