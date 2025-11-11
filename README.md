# ESP32-Robot-Arm-Web-Glove-Control
A comprehensive control system for a 4-axis robotic arm using an ESP32. Features a sophisticated web interface (WebSocket), a wearable glove controller (ESP-NOW), and a motion sequencer.

# ESP32 Robotic Arm Control Suite

![Project Demo Banner](https://user-images.githubusercontent.com/your-username/your-repo/assets/placeholder.png) <!-- It's highly recommended to replace this with a screenshot or GIF of the web interface in action -->

A comprehensive control system for a 4-axis robotic arm built with the ESP32. This project provides two distinct methods for controlling the arm:

1.  **Wearable Glove Controller:** A simple glove with a single flex sensor to control the arm's gripper in real-time using the low-latency ESP-NOW protocol.
2.  **Advanced Web Interface:** A sleek, modern, and responsive web dashboard that connects to the robot arm over Wi-Fi (via WebSocket) for full manual control, motion sequencing, and system monitoring.

The robot arm itself acts as a central hub, creating its own Wi-Fi Access Point and listening for commands from either the glove or any connected web client.

## ✨ Features

### 🤖 Robot Arm Core
- **Dual Control System:** Seamlessly accepts commands from both ESP-NOW (glove) and WebSocket (web UI).
- **Wi-Fi Access Point:** Creates its own Wi-Fi network for easy connection without needing an external router.
- **PCA9685 Servo Driver:** Controls up to 4 servos smoothly and efficiently, offloading the PWM signal generation from the ESP32.
- **Status LED:** Provides visual feedback for initialization status and incoming commands.

### 🌐 Web Command Center (`robot arm .html`)
- **Fully Responsive UI:** Modern interface that works on both desktop and mobile devices.
- **Real-Time Control:** Manual control sliders for each joint and a virtual joystick for intuitive base/shoulder movement.
- **Motion Sequencer:** Create, play, modify, and save complex movement sequences.
- **Save/Load Sequences:** Export your motion sequences to a JSON file and import them later.
- **Dashboard View:** At-a-glance view of system status and real-time joint angles.
- **System Logs:** A terminal-like view for monitoring raw communication messages.
- **Theming:** Switch between light and dark modes.

### 🧤 Glove Controller (`handcon.ino`)
- **ESP-NOW Protocol:** Utilizes Espressif's low-latency, connectionless protocol for quick and responsive control.
- **Flex Sensor Integration:** Reads a flex sensor to intuitively open and close the robot's gripper.
- **Unicast Communication:** Sends data directly to the robot arm's unique MAC address for a reliable link.

## 🏗️ System Architecture

The project consists of three main components that communicate as follows:

1.  **Glove Controller (ESP32)**: Reads the flex sensor and sends gripper commands directly to the Robot Arm via **ESP-NOW**.
2.  **Web Browser (Client)**: Loads the `robot arm .html` file and connects to the Robot Arm's server via **WebSocket** over Wi-Fi.
3.  **Robot Arm (ESP32)**: Acts as the central controller. It creates a Wi-Fi AP, runs a WebSocket server, and listens for ESP-NOW messages. It drives the servos via a **PCA9685** driver.

## 🛠️ Hardware Requirements

#### For the Robot Arm
- 1 x ESP32 DevKit V1
- 1 x PCA9685 16-Channel Servo Driver
- 4 x Servo Motors (e.g., SG90 or MG90S)
- 1 x 4-DOF Robot Arm Chassis
- 1 x 5V Power Supply (2A minimum, capable of powering the ESP32 and all servos)
- Jumper Wires

#### For the Glove Controller
- 1 x ESP32 DevKit V1
- 1 x Flex Sensor
- 1 x 10kΩ Resistor (for the voltage divider with the flex sensor)
- 1 x Glove
- Power source (e.g., LiPo battery or USB power bank)

##  wiring

### 1. Robot Arm Wiring
- **ESP32 to PCA9685:**
  - `GND` -> `GND`
  - `3.3V` -> `VCC`
  - `GPIO 21 (SDA)` -> `SDA`
  - `GPIO 22 (SCL)` -> `SCL`
- **Servos to PCA9685:**
  - Connect the 4 servos to channels `0`, `1`, `2`, and `3`.
- **Power:**
  - Connect the 5V external power supply to the PCA9685's `V+` and `GND` terminals.
  - **Important:** Connect the `GND` of the external power supply to the `GND` of the ESP32 to create a common ground. Power the ESP32 via its USB port or a separate 3.3V/5V pin.

### 2. Glove Controller Wiring
- **Flex Sensor to ESP32:**
  - Create a voltage divider circuit.
  - `ESP32 3.3V` -> One pin of the Flex Sensor
  - The other pin of the Flex Sensor -> `ESP32 GPIO 34`
  - `ESP32 GPIO 34` -> 10kΩ Resistor -> `ESP32 GND`

## ⚙️ Software & Libraries

1.  **Arduino IDE** or **PlatformIO**.
2.  **ESP32 Board Support:** Install the ESP32 boards in your IDE.
3.  **Required Arduino Libraries:**
    - `WiFi.h` (included with ESP32 core)
    - `esp_now.h` (included with ESP32 core)
    - `ESPAsyncWebServer`
    - `AsyncTCP`
    - `Adafruit_PWMServoDriver`
    - `Wire.h` (included with Arduino)

## 🚀 Setup & Installation Guide

### Step 1: Flash the Robot Arm (`ROBOTARM.ino`)

1.  Open `ROBOTARM.ino` in the Arduino IDE.
2.  Select your ESP32 board and COM port.
3.  Upload the code to the ESP32 connected to the PCA9685 servo driver.
4.  Open the **Serial Monitor** at **115200 baud**.
5.  On startup, the ESP32 will print its MAC address. **Copy this MAC address!** It will look like this:
    ```
    ESP MAC: 24:6F:28:AB:CD:EF
    ```

### Step 2: Configure and Flash the Glove Controller (`handcon.ino`)

1.  Open `handcon.ino` in the Arduino IDE.
2.  Find the following line:
    ```cpp
    uint8_t robotMac[] = {0x24, 0x6F, 0x28, 0xAB, 0xCD, 0xEF}; // EXAMPLE MAC
    ```
3.  **Replace the example MAC address** with the one you copied from the robot arm's serial output.
4.  Upload the code to the second ESP32 (the glove controller).

### Step 3: Calibrate the Flex Sensor (Optional but Recommended)

1.  With the glove controller connected to your computer, open the Serial Monitor.
2.  Bend and straighten your finger, observing the "raw" values printed.
    ```
    Sent: seq=1 B=90 S=90 E=90 G=120 (raw 1710) <-- Straight
    Sent: seq=2 B=90 S=90 E=90 G=40 (raw 3095)  <-- Bent
    ```
3.  Update the calibration values in `handcon.ino` to match your sensor's range for better accuracy:
    ```cpp
    #define GRIPPER_FLEX_STRAIGHT 1700 // Your value when finger is straight
    #define GRIPPER_FLEX_BENT 3100     // Your value when finger is bent
    ```
4.  Re-upload the code if you make changes.

## 🎮 How to Use

### Using the Web Interface

1.  Power on the Robot Arm. The built-in LED should blink 3 times, indicating ESP-NOW initialized successfully.
2.  On your computer or phone, search for Wi-Fi networks. Connect to the network named **`RobotArm_AP`** with the password **`12345678`**.
3.  Open a web browser and navigate to **`http://192.168.4.1`**.
4.  The `robot arm .html` file will be served, and the control interface will load.
5.  Click **"Connect to Arm"** and choose the WebSocket option to establish a connection.
6.  You can now use the sliders, joystick, and sequencer to control the arm.

### Using the Glove Controller

1.  Power on both the Robot Arm and the Glove Controller.
2.  The glove controller will automatically start sending commands to the arm via ESP-NOW.
3.  Bend the finger with the flex sensor to close the gripper and straighten it to open the gripper. The arm's LED will blink briefly for each command received.

## 🤝 Contributing

Contributions, issues, and feature requests are welcome! Feel free to check the [issues page](https://github.com/your-username/your-repo/issues).

## 📄 License

This project is licensed under the MIT License. See the `LICENSE` file for details.
