Connecting Pi's to the Tunnel
by William Terry - Thursday, 20 November 2025, 10:38 AM
Number of replies: 0
Some of you have asked how to connect the Raspberry Pis to the tunnel. Normally you could use a USB-A to RX/TX serial cable, but those can be expensive.
A cheaper and reliable alternative is to connect each Raspberry Pi to an STM32, and use the STM32 as a bridge to read the Pi’s serial output and forward it to your PC.

Below is a clear guide, along with a summary of the two test files (one for the Pi, one for the STM32).

Hardware Connection
We only need the Pi’s TX pin and GND:

Raspberry Pi TX: Pin 8 (GPIO14)

Raspberry Pi GND: Pin 6 (or any ground pin)

Connect these to the STM32 as follows:

Pi TX → STM32 RX pin (we’re using PA10 / D2 on the STM board)

Pi GND → STM32 GND

This GND-to-GND link is essential so both devices share a common ground.

Once wired, you can connect the STM32 to your PC via USB as usual. The STM32 will act as a serial pass-through for whatever the Pi transmits.

Software Overview
You mainly need to focus on the Raspberry Pi code, but I’ll provide both files (with the STM32 file in a separate announcment).

Raspberry Pi script:
This simply sends UART output over its TX pin.

STM32 firmware:
This waits for incoming data on the RX pin using an interrupt-based UART handler.
Whenever a byte arrives, the STM32 immediately forwards it to your PC through its USB serial interface.

This lets you monitor the Pi’s TX output without needing a dedicated USB-serial cable.

---

