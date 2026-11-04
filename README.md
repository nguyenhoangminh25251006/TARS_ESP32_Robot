# 🤖 TARS ESP32 Robot - Embodied AI Robotic Assistant

[![ESP-IDF](https://img.shields.io/badge/ESP--IDF-v5.3%2B-blue.svg)](https://github.com/espressif/esp-idf)
[![Hardware](https://img.shields.io/badge/Hardware-ESP32%20%7C%20ESP32--S3-green.svg)](https://www.espressif.com/)
[![MCP](https://img.shields.io/badge/Protocol-Model%20Context%20Protocol-orange.svg)](https://modelcontextprotocol.io/)
[![License](https://img.shields.io/badge/License-MIT-purple.svg)](LICENSE)

**TARS ESP32 Robot** is an open-source, embodied AI robotic assistant inspired by the iconic robot **TARS** from the sci-fi movie *Interstellar*. Built upon the **XiaoZhi ESP32** ecosystem, the robot seamlessly bridges cloud-based Large Language Models (e.g., Qwen, DeepSeek, ChatGPT) with physical bipedal/quad-servo actuation via the **Model Context Protocol (MCP)**, featuring real-time conversational voice interaction and synchronized multi-servo kinematics.

---

## 🌟 Key Features

- 🧠 **Embodied AI with Model Context Protocol (MCP):**
  - Native on-device MCP client and server running directly on the ESP32.
  - The LLM possesses self-body awareness and autonomously schedules physical movement tools (`self.movement.send`) based on conversational context, user intent, or emotional cues.
- 🗣️ **Real-Time Streaming Voice AI:**
  - Offline acoustic wake-word detection powered by [ESP-SR](https://github.com/espressif/esp-sr).
  - Low-latency bi-directional audio streaming using the Opus audio codec over WebSocket / MQTT.
  - End-to-end pipeline: Streaming ASR $\rightarrow$ LLM Reasoning $\rightarrow$ Streaming TTS.
  - Speaker identification and voiceprint recognition ([3D Speaker](https://github.com/modelscope/3D-Speaker)).
- 🦾 **Smooth 4-Servo Gait Kinematics:**
  - Driven by the **PCA9685** 16-channel 12-bit PWM I2C controller (or direct onboard LEDC PWM).
  - Multi-servo synchronized interpolation (`MoveServosSynchronized`) prevents abrupt snapping, reduces mechanical stress, and eliminates gear burnout.
  - Built-in gait patterns: `MOVE_FORWARD`, `MOVE_BACKWARD`, `TURN_LEFT`, `TURN_RIGHT`, `POSE`, `LAUGH` (celebration dance), and `STOP`.
- 🎛️ **Interactive Python Calibration GUI (`calib_gui.py`):**
  - Modern desktop application built with Python Tkinter (Catppuccin dark theme).
  - Real-time angle & PWM trim over USB Serial COM port.
  - One-click non-volatile storage (**NVS Flash**) persistence on ESP32, automatically restored on power-up.
- 📺 **Dynamic Expression Display & Emotions:**
  - Supports round and rectangular LCD/OLED displays (GC9A01, ST7789, SSD1306).
  - Animated facial expressions and emojis dynamically rendered in sync with audio output.

---

## 🛠️ Hardware Architecture & Wiring

### 1. Recommended Bill of Materials (BOM)
- **MCU:** ESP32-S3 (recommended: 16MB Flash, 8MB Octal PSRAM) or standard ESP32.
- **Actuator Controller:** PCA9685 16-Channel 12-bit PWM I2C Module.
- **Servos:** 4x Metal-gear hobby servos (e.g., MG90S, MG996R, SPT4412LV, or equivalent high-torque servos).
- **Audio Input (Microphone):** INMP441 I2S digital microphone (or ES7210 / ES8311 audio board).
- **Audio Output (Amplifier/Speaker):** MAX98357A I2S DAC Amplifier with a 4Ω 3W speaker.
- **Display:** GC9A01 (round 240x240 SPI LCD) or ST7789 (rectangular 240x240/320 SPI LCD).
- **Power Supply:** Dedicated external 5V 3A–5A supply for servos (always connect **common GND** with ESP32).

---

### 2. Wiring & Pinout

#### PCA9685 I2C Interface to ESP32:
| PCA9685 Pin | ESP32 Pin (Default) | Notes |
| :--- | :--- | :--- |
| **VCC** | 3.3V or 5V | Logic power |
| **GND** | GND | Common system ground |
| **SDA** | **GPIO 21** | I2C Data line |
| **SCL** | **GPIO 22** | I2C Clock line |
| **V+ (Servo)** | External 5V (3A+) | Servo motor power |

#### PCA9685 Servo Channels:
| Channel | Servo Identifier | Mechanical Function |
| :---: | :--- | :--- |
| **CH 0** | **Left Height (LH)** | Controls left body vertical elevation |
| **CH 1** | **Right Height (RH)** | Controls right body vertical elevation |
| **CH 2** | **Left Leg (LL)** | Controls left leg swing / stride |
| **CH 3** | **Right Leg (RL)** | Controls right leg swing / stride |

---

## 📂 Repository Structure

```text
TARS_ESP32_Robot/
├── calib_gui.py               # Interactive desktop GUI for servo calibration via Serial
├── CMakeLists.txt             # Top-level ESP-IDF CMake configuration
├── sdkconfig.defaults*        # Target default configurations for ESP32/S3/C3
├── partitions/                # Partition tables (Flash memory layout)
├── docs/                      # Architectural specs, MCP protocol docs, diagrams
├── scripts/                   # Asset generation, audio conversion, and deployment scripts
└── main/
    ├── application.cc / .h    # Core application loop & event orchestrator
    ├── mcp_server.cc / .h     # Device MCP tool registry (e.g. self.movement.send)
    ├── movement/              # Robotic locomotion and servo subsystem
    │   ├── pca9685.cc / .h    # I2C driver for PCA9685 PWM controller
    │   ├── servo_controller.cc# Gaits, trajectory smoothing, and NVS storage
    │   ├── servo_controller.h # Class declaration, default angles, and kinematics
    │   └── movement_commands.h# Movement command enums and string mappings
    ├── audio/                 # I2S audio drivers, Opus encoder/decoder
    ├── boards/                # Board support packages (BSP) for various devkits
    └── display/               # LVGL graphics engine, facial emojis, animations
```

---

## 🚀 Getting Started

### 1. Prerequisites
- **ESP-IDF:** v5.3 or higher (tested on **ESP-IDF v5.5.1**).
  Follow the official [Espressif ESP-IDF Installation Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/).
- **Python:** 3.10+ with `pip`.

### 2. Clone the Repository
```bash
git clone https://github.com/nguyenhoangminh25251006/TARS_ESP32_Robot.git
cd TARS_ESP32_Robot
```

### 3. Set Target & Configure
Select your target hardware chip (e.g., ESP32-S3):
```bash
idf.py set-target esp32s3
```

Configure Wi-Fi, audio pinout, and backend endpoint if necessary:
```bash
idf.py menuconfig
```

### 4. Build and Flash
Connect your ESP32 board to your computer via USB:
```bash
# Compile the firmware
idf.py build

# Flash and open serial monitor (replace COMx with your serial port, e.g. COM3 or /dev/ttyUSB0)
idf.py -p COMx flash monitor
```

---

## 📐 Servo Calibration Guide (`calib_gui.py`)

Precise calibration of neutral positions is critical for stable locomotion. TARS includes an intuitive Python GUI for real-time calibration:

1. Connect the ESP32 to your PC via USB.
2. Install the required dependency:
   ```bash
   pip install pyserial
   ```
3. Launch the calibration application:
   ```bash
   python calib_gui.py
   ```
4. Select your **COM Port** and baud rate (`115200`), then click **Connect**.
5. Adjust the sliders in real time:
   - **LH (Left Height):** Left hip/torso vertical alignment.
   - **RH (Right Height):** Right hip/torso vertical alignment.
   - **LL (Left Leg):** Left leg neutral center angle.
   - **RL (Right Leg):** Right leg neutral center angle.
6. Click **Save to NVS**. The neutral trim parameters are saved directly into the ESP32's non-volatile flash memory and automatically loaded on every system boot.

---

## 💬 Voice & AI Commands

With the **Model Context Protocol (MCP)**, the AI understands natural language intent in multiple languages (English, Vietnamese, Chinese, etc.) and triggers the corresponding physical actions:

| Voice Prompt (English) | Vietnamese Equivalent | Triggered MCP Action |
| :--- | :--- | :--- |
| *"TARS, walk forward!"* | *"TARS ơi, tiến lên phía trước"* | `self.movement.send(command="MOVE_FORWARD")` |
| *"Step back a bit"* | *"Lùi lại một chút nào"* | `self.movement.send(command="MOVE_BACKWARD")` |
| *"Turn left / Turn right"* | *"Rẽ sang trái / Rẽ sang phải"* | `self.movement.send(command="TURN_LEFT" / "TURN_RIGHT")` |
| *"Strike a pose!"* | *"Tạo dáng chụp ảnh xem nào"* | `self.movement.send(command="POSE")` |
| *"Tell me a funny joke"* | *"Kể chuyện cười đi"* | Speaks joke while performing celebratory motion (`LAUGH`) |
| *"Halt / Freeze"* | *"Dừng lại ngay"* | `self.movement.send(command="STOP")` |

---

## 🤝 Contributing

Contributions, bug reports, and pull requests to enhance TARS gaits, add inverse kinematics (IK), or expand MCP tools are warmly welcomed!

---

## 📜 License

This project is licensed under the [MIT License](LICENSE). Special thanks to the open-source **XiaoZhi ESP32** community and **Espressif Systems**.
