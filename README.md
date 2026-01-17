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
pip install -r requirements.txt
```

### Arduino Libraries
* `Adafruit ADS1X15`

## 🚀 Getting Started

**Note:** For detailed wiring diagrams, pin configurations, and advanced tuning, please refer to the **[manual.pdf](manual.pdf)** included in this repository.

### 1. Flash the Firmware
1.  Open the `.ino` file in the Arduino IDE.
2.  Verify the `PIN_DEFINITIONS` match your physical wiring.
3.  Upload the code to your board.
4.  **Important:** Close the Arduino Serial Monitor before proceeding.

### 2. Launch the Control System
1.  Navigate to the project directory.
2.  Run the main script:
    ```bash
    python main.py
    ```
3.  Select your COM port from the dropdown and click **Connect**.
4.  Use the "Trajectory Editor" to draw your gait or load a preset JSON file.

## ⚙️ Control Strategy

The system utilizes a master-slave communication protocol:
1.  **Python (Master):** Sends target pressure vectors to the microcontroller at a user-defined frequency.
2.  **Arduino (Slave):**
    * Reads the XGZP6847 sensors via I2C.
    * Executes the **Gain Scheduled State Machine**:
        * *High Error State:* Aggressive valve opening for fast filling/venting.
        * *Low Error State:* Micro-pulsing for precise target convergence.
        * *Hold State:* Valves closed (Deadband).
    * Returns actual pressure telemetry to the UI.

## 📄 Documentation

A comprehensive user manual is available: **[manual.pdf](manual.pdf)**.
It covers:
* Electrical Schematics
* Gait Design Strategies (Inchworm locomotion)
* Troubleshooting Communication Deadlocks
* Calibration of Pressure Sensors

---
*Developed for the [Master Thesis in Takemura Laboratory at Keio University]*