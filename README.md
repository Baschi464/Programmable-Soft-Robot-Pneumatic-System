# Modular Soft Robot Pneumatic Controller

![Python](https://img.shields.io/badge/Python-3.x-blue) ![Arduino](https://img.shields.io/badge/Firmware-Arduino-teal) ![License](https://img.shields.io/badge/License-MIT-green)

A universal, open-source control stack for pneumatic soft robots. This system provides precise pressure regulation for multi-chamber actuators, combining a high-performance Python dashboard with robust microcontroller firmware.

While currently configured for an **8-channel omnidirectional inchworm robot**, the GUI can be configured to drive systems ranging from single-actuator grippers to complex 20-channel walkers.



## ✨ Key Features

* **Scalable Architecture:** The Python GUI dynamically adjusts to control between 1 and 20 independent pressure channels.
* **Trajectory Editor:** Visual "Click-to-Edit" graph interface for designing complex gaits (e.g., peristaltic motion, grasping sequences).
* **Advanced Control Logic:** Implements a **State Machine Controller with Gain Scheduling**, optimizing valve response times based on pressure error magnitude and inflation/deflation states.
* **Real-Time Telemetry:** Live visualization of pressure data with adjustable time windows.
* **Data Logging:** One-click CSV export with smart slicing (Last 10s, 30s, Custom) for post-processing in MATLAB/Excel.

## 🛠️ Hardware Specifications

This repository is built for the following reference hardware stack:

* **Microcontroller:** Arduino (Mega/Uno compatible)
* **ADC:** 2x Adafruit ADS1115 (16-bit precision)
* **Pressure Sensors:** **CFSensors XGZP6847A100KPGPN** (0-100 kPa)
* **Actuation:**
    * Binary Solenoid Valves (3/2 way)
    * Dual Master Pump Configuration (Positive Pressure / Vacuum)

## 💻 Software Requirements

### Python Dependencies
* Python 3.x
* `pyserial` (Serial Communication)
* `matplotlib` (Real-time plotting)

Install via pip:
```bash
pip install pyserial matplotlib

