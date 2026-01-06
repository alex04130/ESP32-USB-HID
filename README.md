# ESP32-S3 USB HID UPS (Driverless UPS on Windows/macOS)

This project implements a **standard USB HID UPS** device on **ESP32-S3** using **ESP-IDF + TinyUSB**.
It is recognized by **Windows/macOS without any driver**, reports battery/charging state, and supports OS power events (e.g., safe shutdown policy).

## Why it matters
- Bridges **power electronics + embedded firmware + OS integration**
- Focuses on **reliability engineering** (board-level flashing consistency + architecture redesign)

## Features
- Standard **HID Power Device / UPS** report descriptor
- I²C integration with power/battery ICs (SW7203 + BQ4050)
- Status LEDs (error/charging/power-loss)
- Tested: Windows, macOS (Linux expected to work)

## Hardware Architecture (High-level)
- Battery pack: 4S 21700
- NVDC power path: SW7203
- Battery metering/protection: BQ4050
- USB device + control: ESP32-S3 (WROOM)

> Add a block diagram here (image or Mermaid).

## Build & Flash
Requirements:
- ESP-IDF (matching your project version)

Steps:
```bash
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

## Project Structure
 - main/ - application entry and device integration
 - include/esp32ups_hid/ - HID report descriptor & UPS definitions
 - include/bq4050/ - BQ4050 access layer
 - include/SK_BQ4050_HID/ - mapping between metering and HID fields

## Validation Notes
Verified driverless enumeration and UPS recognition on Windows/macOS
Earlier hardware revision exhibited idf.py flashing failure near ~66% write progress.
A controlled experiment (desoldered module + external USB-UART) isolated the issue to PCB physical implementation, leading to a full redesign (v1.2).

## License
(TODO)
