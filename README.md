# 🤖 TARS ESP32 Robot - Embodied AI Robotic Assistant

[![ESP-IDF](https://img.shields.io/badge/ESP--IDF-v5.3%2B-blue.svg)](https://github.com/espressif/esp-idf)
[![Hardware](https://img.shields.io/badge/Hardware-ESP32%20%7C%20ESP32--S3-green.svg)](https://www.espressif.com/)
[![MCP](https://img.shields.io/badge/Protocol-Model%20Context%20Protocol-orange.svg)](https://modelcontextprotocol.io/)
[![License](https://img.shields.io/badge/License-MIT-purple.svg)](LICENSE)

Dự án **TARS ESP32 Robot** là một robot thông minh tích hợp AI thể nghiệm (Embodied AI) lấy cảm hứng từ robot **TARS** trong bộ phim khoa học viễn tưởng *Interstellar*. Dự án được phát triển dựa trên nền tảng **Xiaozhi ESP32**, kết hợp khả năng giao tiếp giọng nói thời gian thực với các mô hình ngôn ngữ lớn (LLM như Qwen, DeepSeek, ChatGPT) thông qua giao thức **MCP (Model Context Protocol)** để điều khiển hành vi và chuyển động vật lý bằng 4 động cơ servo.

---

## 🌟 Tính Năng Nổi Bật (Key Features)

- 🧠 **Trí tuệ nhân tạo thể nghiệm (Embodied AI):** 
  - Tích hợp giao thức **Model Context Protocol (MCP)** trực tiếp trên ESP32.
  - LLM có khả năng nhận thức cơ thể vật lý và tự động gọi công cụ `self.movement.send` để di chuyển, tạo dáng hoặc biểu đạt cảm xúc theo ngữ cảnh trò chuyện.
- 🗣️ **Tương tác giọng nói thời gian thực (Real-time Voice AI):**
  - Đánh thức bằng giọng nói offline (ESP-SR Wake-word).
  - Truyền phát âm thanh hai chiều chuẩn nén Opus qua WebSocket / MQTT.
  - Xử lý hội thoại nối tiếp ASR + LLM + TTS với độ trễ cực thấp.
  - Nhận diện người nói (Speaker Identification - 3D Speaker).
- 🦾 **Điều khiển chuyển động 4 Servo mượt mà (Smooth Motion Control):**
  - Sử dụng mạch mở rộng PWM 16 kênh **PCA9685 (I2C)** hoặc GPIO onboard.
  - Thuật toán nội suy đồng bộ nhiều servo (`MoveServosSynchronized`), tránh giật cục và bảo vệ bánh răng servo.
  - Các dáng di chuyển định sẵn: Đi tới (`MOVE_FORWARD`), Đi lùi (`MOVE_BACKWARD`), Rẽ trái (`TURN_LEFT`), Rẽ phải (`TURN_RIGHT`), Tự giới thiệu/Tạo dáng (`POSE`), Cười đùa/Lắc lư (`LAUGH`), Dừng lại (`STOP`).
- 🎛️ **Công cụ cân chỉnh Servo trực quan (`calib_gui.py`):**
  - Giao diện Python GUI (Tkinter/Catppuccin theme) kết nối qua Serial COM Port.
  - Điều chỉnh góc và PWM trực tiếp theo thời gian thực (Real-time Tuning).
  - Lưu cấu hình vĩnh viễn vào bộ nhớ flash **NVS (Non-Volatile Storage)** của ESP32.
- 📺 **Màn hình biểu cảm cảm xúc (Display & Emotions):**
  - Hỗ trợ màn hình LCD (ST7789, GC9A01) / OLED hiển thị biểu cảm khuôn mặt sống động tương ứng với giọng nói và hành động.

---

## 🛠️ Cấu Trúc Phần Cứng (Hardware Architecture)

### 1. Danh sách linh kiện đề xuất
- **Vi điều khiển:** ESP32-S3 (khuyên dùng bản 16MB Flash, 8MB PSRAM) hoặc ESP32.
- **Mạch lái Servo:** Mạch PCA9685 I2C 16-Channel 12-bit PWM.
- **Động cơ Servo:** 4x Servo kim loại (MG90S, MG996R, SPT4412LV hoặc servo tương đương).
- **Âm thanh đầu vào (Microphone):** INMP441 I2S Microphone (hoặc ES7210 / ES8311 audio board).
- **Âm thanh đầu ra (Speaker/DAC):** Mạch khuếch đại I2S MAX98357A + Loa 4Ω 3W.
- **Màn hình hiển thị:** Màn hình tròn GC9A01 hoặc chữ nhật ST7789 (SPI).
- **Nguồn cấp:** Nguồn 5V 3A - 5A riêng cho Servo (chú ý **nối chung GND** với ESP32).

### 2. Sơ đồ đấu nối (Pinout & Wiring)

#### Mạch PCA9685 (I2C) nối với ESP32:
| Chân PCA9685 | Chân ESP32 (Mặc định) | Ghi chú |
| :--- | :--- | :--- |
| **VCC** | 3.3V hoặc 5V | Nguồn logic |
| **GND** | GND | Nối chung mass toàn hệ thống |
| **SDA** | **GPIO 21** | Chân I2C SDA |
| **SCL** | **GPIO 22** | Chân I2C SCL |
| **V+ (Servo)** | 5V Nguồn ngoài | Cung cấp tải cho 4 Servo |

#### Thứ tự kênh Servo trên PCA9685:
| Kênh (Channel) | Tên Servo | Chức năng trên Robot TARS |
| :---: | :--- | :--- |
| **CH 0** | **Left Height (LH)** | Servo nâng hạ thân bên trái |
| **CH 1** | **Right Height (RH)** | Servo nâng hạ thân bên phải |
| **CH 2** | **Left Leg (LL)** | Servo chân/càng xoay bên trái |
| **CH 3** | **Right Leg (RL)** | Servo chân/càng xoay bên phải |

---

## 📂 Cấu Trúc Dự Án (Project Structure)

```text
TARS_ESP32_Robot/
├── calib_gui.py               # Công cụ Python GUI cân chỉnh góc servo qua Serial
├── CMakeLists.txt             # Cấu hình biên dịch ESP-IDF Project
├── sdkconfig.defaults*        # Cấu hình sdkconfig cho các dòng chip ESP32/S3
├── partitions/                # Bảng phân vùng bộ nhớ ESP32 (Flash partitions)
├── docs/                      # Tài liệu kỹ thuật, sơ đồ, kiến trúc MCP
├── scripts/                   # Script hỗ trợ flash và build
└── main/
    ├── application.cc / .h    # Khởi tạo ứng dụng & luồng điều phối chính
    ├── mcp_server.cc / .h     # Đăng ký các công cụ MCP cho AI (self.movement.send)
    ├── movement/              # Phân hệ điều khiển chuyển động của Robot
    │   ├── pca9685.cc / .h    # Driver giao tiếp I2C với IC PCA9685
    │   ├── servo_controller.cc# Xử lý dáng đi, đồng bộ góc và lưu trữ NVS
    │   ├── servo_controller.h # Khai báo class, thông số PWM và các tư thế
    │   └── movement_commands.h# Định nghĩa tập lệnh di chuyển (enum & string)
    ├── audio/                 # Trình điều khiển codec âm thanh và I2S
    ├── boards/                # Cấu hình phần cứng theo từng loại kit dev
    └── display/               # Xử lý render giao diện, biểu cảm khuôn mặt
```

---

## 🚀 Hướng Dẫn Cài Đặt & Biên Dịch (Getting Started)

### Bước 1: Cài đặt môi trường ESP-IDF
Dự án tương thích tối ưu với **ESP-IDF v5.3+** (khuyên dùng ESP-IDF v5.5.1).
Tham khảo hướng dẫn cài đặt chính thức của Espressif: [ESP-IDF Get Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/).

### Bước 2: Clone mã nguồn
```bash
git clone https://github.com/nguyenhoangminh25251006/TARS_ESP32_Robot.git
cd TARS_ESP32_Robot
```

### Bước 3: Thiết lập mục tiêu chip và nạp cấu hình
Chọn chip bạn đang sử dụng (ví dụ ESP32-S3):
```bash
idf.py set-target esp32s3
```

Tùy chỉnh cấu hình (WiFi, Server backend, chân pin nếu cần):
```bash
idf.py menuconfig
```

### Bước 4: Biên dịch và nạp firmware
Cắm kit ESP32 vào máy tính qua cổng USB:
```bash
# Biên dịch dự án
idf.py build

# Nạp firmware và mở Serial Monitor (thay COMx bằng cổng của bạn)
idf.py -p COMx flash monitor
```

---

## 📐 Cân Chỉnh Động Cơ Servo (Calibration Guide)

Để robot TARS đứng thẳng và bước đi chuẩn xác, cần cân chỉnh góc neutral (tọa độ trung tính) cho 4 servo thông qua phần mềm trực quan:

1. Kết nối ESP32 với máy tính qua cổng Serial USB.
2. Cài đặt thư viện phụ thuộc cho Python (nếu chưa có):
   ```bash
   pip install pyserial
   ```
3. Chạy phần mềm cân chỉnh:
   ```bash
   python calib_gui.py
   ```
4. Chọn đúng cổng COM và tốc độ Baud (thường là `115200`), nhấn **Connect**.
5. Kéo các thanh trượt để tinh chỉnh từng servo:
   - **LH (Left Height):** Độ cao thân trái
   - **RH (Right Height):** Độ cao thân phải
   - **LL (Left Leg):** Góc xoay chân trái
   - **RL (Right Leg):** Góc xoay chân phải
6. Nhấn **Save to NVS** để lưu trực tiếp vào bộ nhớ flash của ESP32. Các giá trị này sẽ tự động nạp mỗi khi robot khởi động lại!

---

## 💬 Tương Tác Qua Trí Tuệ Nhân Tạo (AI Interaction)

Nhờ tích hợp **Model Context Protocol (MCP)**, bạn có thể nói chuyện tự nhiên bằng tiếng Việt hoặc tiếng Anh với robot. AI sẽ tự động phân tích ý định và kích hoạt chuyển động:

- *"TARS ơi, tiến lên phía trước đi!"* ➡️ Gọi tool `self.movement.send(command="MOVE_FORWARD")`
- *"Lùi lại một chút nào"* ➡️ Gọi tool `self.movement.send(command="MOVE_BACKWARD")`
- *"Quay sang trái / Quay sang phải"* ➡️ Gọi tool `TURN_LEFT` / `TURN_RIGHT`
- *"Tạo dáng chụp ảnh xem nào"* ➡️ Gọi tool `POSE`
- *"Kể chuyện vui gì đi"* ➡️ Robot vừa nói vừa thực hiện động tác nhún nhảy `LAUGH`

---

## 🤝 Đóng Góp & Phát Triển (Contribution)

Mọi đóng góp, báo lỗi (Issues) và Pull Requests cải tiến các dáng đi mới cho TARS đều rất được hoan nghênh!

---

## 📜 Giấy Phép (License)

Dự án phát hành dưới giấy phép [MIT License](LICENSE). Trân trọng cảm ơn cộng đồng mã nguồn mở **Xiaozhi ESP32** và **Espressif Systems**.
