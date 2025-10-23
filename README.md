# Project J1 - CV Robotics Platform


This is J1, a custom-built tracked robot I designed and built from scratch. It's designed to be a flexible platform for computer vision and robotics experiments, running on a Raspberry Pi 5 with support from a Pico W.

## 🧠 Project Goal

The main goal is to create a capable mobile robot that can be driven manually now, but is architected with future autonomy (like SLAM and navigation) in mind. It integrates sensor data, controller input, servo control, and a visual display showing the robot's status and a simple face.

---

## ✅ Current Status (As of Oct 2025)

* **Hardware:** Fully assembled! The chassis, tracks, servos, sensors, compute modules, and display are all integrated.
* **Manual Control:** The robot can be driven using a Nintendo Switch Pro Controller via Bluetooth. `control.py` reads the controller and sends movement/head commands to the Pico W over UART.
* **Sensor Integration:** The `sensors.py` script successfully reads data from all onboard sensors (IMU, environmentals, ADC, power monitor) and logs it continuously to a JSON file (`/dev/shm/sensor_data.json`).
* **Live Display:** The `display.py` script runs on the main DSI screen, showing a robot face and a status display. It reads the sensor data from the JSON file to update relevant fields (temperature, pressure, humidity, battery, etc.) in real-time. It also displays a live feed from the camera.
* **Basic Communication:** RPi 5 <-> Pico W communication via UART is functional for sending commands and receiving basic acknowledgments.

---

## 🧩 Hardware Overview

* **Compute:** Raspberry Pi 5 (16GB), Raspberry Pi Pico W
* **AI:** Raspberry Pi AI Hat+ (Hailo-8) *(Note: AI features not yet implemented)*
* **Vision:** Arducam 64MP Owlsight Camera
* **Display:** Waveshare 4.3" DSI LCD
* **Sensors:** Waveshare Sense HAT (C) (IMU, Env, Light, ADC), Waveshare UPS 3S (INA219 Power Monitor)
* **Mobility:** Custom designed Gobilda Track System, 4x Gobilda 2000 Series Servos, Gobilda PDB
* **Power:** Gobilda 6V NiMH Battery Pack, UPS 3S for RPi power
* **Input:** Switch Pro Controller (Bluetooth)

---

## 🧪 Software Architecture

The system runs across three main Python scripts on the Raspberry Pi 5:

1.  **`sensors.py`:** Dedicated to polling all sensors and broadcasting their state via `/dev/shm/sensor_data.json`.
2.  **`control.py`:** Handles controller input and sends commands to the Pico W via UART. Also listens for basic terminal commands. (Planned: Will also broadcast its state via `/dev/shm/control_state.json` and listen for commands via a socket).
3.  **`display.py`:** Renders the Pygame display (face + status info), reads the state JSON files, displays the camera feed, and listens for terminal commands to change the face's state. (Planned: Will listen for face state commands via a socket and include a UI to send commands to `control.py`'s socket).

A **MicroPython** script runs on the **Pico W** to receive UART commands and directly manage the PWM signals for the track/head servos and GoBilda LED headlight.

**Communication Flow:**
* **State:** `sensors.py` -> JSON -> `display.py`. (Planned: `control.py` -> JSON -> `display.py`)
* **Commands:** Controller/Terminal -> `control.py` -> UART -> Pico W.
* **Feedback:** Pico W -> UART -> `control.py` (for basic ACKs/status).
* **(Planned):** `display.py`/`animate.py`/`nav.py` -> Socket -> `control.py` (for sending commands).
* **(Planned):** `animate.py`/`nav.py` -> Socket -> `display.py` (for setting face state).

---

## 🚀 Future Goals

1.  **Control State JSON:** Implement the JSON output in `control.py`.
2.  **Socket IPC:** Replace file-based/terminal command injection with simple TCP sockets for inter-script commands (UART commands to `control.py`, face state changes to `display.py`).
3.  **Animation System:** Create `animate.py` to send timed sequences of pose and face commands via sockets.
4.  **Autonomous Navigation:** Implement SLAM and path planning using sensor data, sending commands via the control socket.
5.  **AI/CV Integration:** Utilize the Hailo accelerator for tasks like object recognition or tracking.

---

## 🛠️ Development Notes

* Main scripts use Python 3.11+. Pico uses MicroPython.
* Libraries: Pygame, PySerial, Picamera2, NumPy, lgpio, python3-smbus, JSON.
