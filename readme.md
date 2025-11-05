# 🤖 J1 Robotics Platform: Decoupled Control and Real-Time AI Architecture

---

## 💡 Project Thesis & Core Challenge

The J1 Robotics Platform is a Raspberry Pi 5-based mobile robot built as a demonstration of complex **computer vision and robotics systems integration**.

The primary architectural challenge was achieving **stable, low-latency, and simultaneous performance** across three critical domains on a single-board computer:

1.  **AI Acceleration:** Real-time object detection using the Hailo-8 NPU.
2.  **Web Control:** Persistent, low-latency teleoperation accessible from any network device.
3.  **Local Headless Display:** Continuous, dedicated video feed for the on-board UI.

The solution utilizes an **Asynchronous, Multi-Process Architecture** where system stability is prioritized over monolithic design.

---

## 🛠️ Hardware Specification (J1 v1.0)

The platform is designed around maximizing the capabilities of the Raspberry Pi 5 and its dedicated accelerator hardware.

| Component | Function | Interface / Protocol |
| :--- | :--- | :--- |
| **Main Compute** | Raspberry Pi 5 (8GB) | Host for all Python microservices (Linux) |
| **AI Accelerator** | Hailo-8 M.2 Edge AI Module | PCIe Gen 2 Interface (4 GT/s) |
| **Motor Controller** | Raspberry Pi Pico (RP2040) | USB-Serial (Pyserial) |
| **Environmental Sensor** | SHTC3 (Temp/Humidity) | I2C Bus (LGPIO) |
| **Barometer** | LPS22HB (Pressure) | I2C Bus (SMBus) |
| **IMU (Body)** | QMI8658 / AK09918 (9-DoF) | I2C Bus (SMBus) |
| **Power Monitor** | INA219 (UPS/Battery) | I2C Bus (SMBus) |
| **ADC** | SGM58031 (Analog Inputs) | I2C Bus (SMBus) |
| **Color Sensor** | TCS34087 (Color/Ambient Light) | I2C Bus (SMBus) |
| **Actuators** | DC Motors w/ Gearbox, Head Servos | PWM/GPIO (Pico) |

---

## ⚙️ Technical Architecture & Inter-Process Communication (IPC)

The robot is managed by **seven decoupled Python microservices** that communicate using high-speed OS primitives, ensuring high throughput and resilience against single-component failure.

| Script File | Primary Responsibility | IPC Protocol(s) | Key Libraries |
| :--- | :--- | :--- | :--- |
| **`robot_operator.py`** | **State Orchestrator & Command Router.** Manages high-level robot state (MANUAL/AUTO) and prevents command conflicts. | TCP Sockets | `selectors`, `threading` |
| **`visionstreamer.py`** | **Triple-Output Vision Pipeline.** Splits the camera feed for all downstream consumers. Performs Hailo inference and publishes the result. | GStreamer, UDP, NumPy Shared Memory | **HailoRT**, `numpy` |
| **`webserver.py`** | **External Communication Bridge.** Serves the HTML frontend and converts the UDP video stream into a stable HTTP MJPEG stream. | WebSockets, TCP/UDP Sockets | `websockets`, `asyncio` |
| **`control.py`** | **Hardware Control Interface.** Translates high-level commands into serial signals for the motor controller (Pico/RP2040). Handles game controller input. | Pygame (Joystick), Serial Port, JSON | `pyserial`, `pygame` |
| **`display.py`** | **Local UI/HMI.** Renders the LCARS dashboard, reading system performance and displaying the raw video feed from shared memory. | JSON/NumPy Shared Memory | `pygame`, `psutil` |
| **`sensors.py`** | **Data Ingestion.** Polling I2C sensors (IMU, Environmental, Power) and publishing raw data to shared state files. | I2C Bus, JSON Shared File | `lgpio`, `smbus` |
| **`speech.py`** | **Text-to-Speech Output.** Processes vocalization commands from the Operator. | TCP Sockets | `pyttsx3` |

---

## 🔬 Execution and Stability

### High-Integrity Video Pipeline
The core execution challenge was solved by building a **GStreamer pipeline using `tee` elements**, which is the **most resource-efficient way** to clone a raw camera source. This technique guarantees the Hailo accelerator receives its data at the highest priority, while the web stream and local display receive separate, stable branches.

### Decoupling State
To prevent command conflicts and state jumping (e.g., when switching from game controller to web UI), the system relies on two critical patterns:
* **Atomic Shared Memory:** Status data (`control_state.json`) and raw video frames (`pygame_frame.npy`) are written to files using atomic operations (`os.rename` or direct `numpy.save`), ensuring that consumers (like `display.py`) never read a corrupted or half-written file during a race condition.
* **Decoupled Servoing:** Relative motion commands from the web UI are calculated by the **`robot_operator.py`** (the "Brain") only after it reads the robot's *actual* current position from the `control.py` status file, eliminating state desynchronization.

### Launch Management
The entire environment stack, including the specialized Hailo environment, is launched via a single **`start_robot.sh`** script managed by $\text{tmux}$. This ensures **process persistence**—allowing all services to run reliably in the background regardless of SSH session status—and guarantees that each service is activated with its exact required environmental variables.

---

## 📅 Future Goals & Development Roadmap

The platform's current success provides a stable foundation for implementing more advanced robotics capabilities. Future development will focus on transitioning the J1 robot from a teleoperated platform to an autonomous agent capable of environmental awareness and navigation.

### I. Autonomous Capabilities & Control

| Goal | Description | Architectural Impact |
| :--- | :--- | :--- |
| **Complete Autonomous Logic** | Fully develop the placeholder logic within `robot_operator.py` to enable complex decision-making based on sensor and vision data. | Refine state machine logic, integrating real-time object/power status for self-preservation and task execution. |
| **Local Velocity Control** | Integrate the track encoders into `control.py` to move from open-loop speed commands to closed-loop velocity control for improved odometry. | Requires PID control loop logic within the motor control firmware (Pico/RP2040) and data streaming back to the host. |
| **Head-IMU Integration** | Integrate the new IMU directly onto the head servo assembly to precisely track the head's orientation and map visual data to a known coordinate frame. | Requires adding `IMU` reading logic to `control.py` or a dedicated head sensor script. |

### II. Sensor Fusion & Mapping (SLAM)

| Goal | Sensor Suite | Purpose |
| :--- | :--- | :--- |
| **Lidar Integration** | **YDLIDAR T-Mini+** (2D Laser Scanner) | Establish reliable 2D environmental sensing, collision avoidance, and feature extraction for simultaneous localization and mapping (SLAM). |
| **Depth/Proximity Sensing** | **Multizone Time-of-Flight (ToF) Imager** | Provide close-range, multi-point obstacle detection and ground plane estimation for near-field navigation, supplementing the wide-range Lidar. |
| **Enhanced Odometry** | **Track Encoders** | Improve dead reckoning accuracy by directly measuring wheel rotation, significantly reducing drift in the localization pipeline. |
| **Full SLAM Implementation** | Integrate Lidar, ToF, and enhanced odometry data into a unified framework (e.g., ROS/SLAM Toolbox, or custom solution) to create and maintain an internal map of the operating environment. | This requires a dedicated fusion node to merge the three sensor inputs, enabling the robot to navigate unknown spaces autonomously. |