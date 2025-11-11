/*
  - Compatible with ESP32 DevKit V1
  - Controls 4 servos via PCA9685 16ch driver
  - Accepts commands via:
      • ESP-NOW (hand controller) - Blinks LED on receive
      • WebSocket (from your external web app)
  - LED on GPIO 2: Blinks on ESP-NOW init success (3x), and on each receive (1x short blink)
*/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ====== LED Setup (Built-in on GPIO 2) ======
#define LED_PIN 2
void blinkLED(int times, int duration = 100) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(duration);
    digitalWrite(LED_PIN, LOW);
    if (i < times - 1) delay(duration);
  }
}

// ====== PCA9685 Setup ======
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

#define SERVO_FREQ 50 // 50Hz for standard servos
#define N_SERVOS 4
int servoAngles[N_SERVOS] = {90, 90, 90, 90};
const int servoChannels[N_SERVOS] = {0, 1, 2, 3}; // PCA9685 channels

// ====== WiFi + WebSocket Setup ======
const char* ssid = "RobotArm_AP";
const char* password = "12345678";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ====== ESP-NOW Command Structure ======
typedef struct {
  uint16_t seq;
  uint8_t base;
  uint8_t shoulder;
  uint8_t elbow;
  uint8_t gripper;
  uint8_t battery;
} __attribute__((packed)) ArmCommand;

ArmCommand lastCmd;

// ====== Servo Helper Functions ======
uint16_t angleToPulse(int angle) {
  // Map 0–180° to 500–2500 µs (PCA9685 uses 4096 steps)
  int pulse_us = map(angle, 0, 180, 500, 2500);
  return (uint16_t)((pulse_us * 4096L) / (1000000L / SERVO_FREQ));
}

void writeServo(int index, int angle) {
  angle = constrain(angle, 0, 180);
  uint16_t pulse = angleToPulse(angle);
  pwm.setPWM(servoChannels[index], 0, pulse);
  servoAngles[index] = angle;
}

// ====== Reset Arm Position ======
void resetPosition() {
  Serial.println("Reset -> 90° all servos");
  for (int i = 0; i < N_SERVOS; i++) {
    writeServo(i, 90);
    delay(20);
  }
}

// ====== ESP-NOW Receive Callback ======
void onRecv(const esp_now_recv_info *recv_info, const uint8_t *incomingData, int len) {
  if (len != sizeof(ArmCommand)) return;
  memcpy(&lastCmd, incomingData, sizeof(ArmCommand));
  Serial.printf("ESP-NOW: seq=%d B=%d S=%d E=%d G=%d\n",
                lastCmd.seq, lastCmd.base, lastCmd.shoulder, lastCmd.elbow, lastCmd.gripper);

  // Blink LED on receive
  blinkLED(1, 100);

  writeServo(0, lastCmd.base);
  writeServo(1, lastCmd.shoulder);
  writeServo(2, lastCmd.elbow);
  writeServo(3, lastCmd.gripper);
}

// ====== WebSocket Message Handler ======
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo *)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    data[len] = 0;
    String msg = (char*)data;
    Serial.println("WS Msg: " + msg);

    if (msg == "RESET") {
      resetPosition();
      ws.textAll("OK:RESET");
    } else if (msg.startsWith("SET:")) {
      int idx, ang;
      if (sscanf(msg.c_str(), "SET:%d:%d", &idx, &ang) == 2) {
        if (idx >= 0 && idx < N_SERVOS && ang >= 0 && ang <= 180) {
          writeServo(idx, ang);
          ws.textAll("OK:SET");
        }
      }
    }
  }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
             AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.println("WebSocket client connected");
    client->text("Connected to ESP32 Robot Arm");
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.println("WebSocket client disconnected");
  } else if (type == WS_EVT_DATA) {
    handleWebSocketMessage(arg, data, len);
  }
}

// ====== Setup ======
void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 Robot Arm (PCA9685 + WebSocket + ESP-NOW)");

  // Initialize LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Initialize I2C and PCA9685
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);

  // Set servos to initial angles
  for (int i = 0; i < N_SERVOS; i++) writeServo(i, servoAngles[i]);

  // Wi-Fi Access Point
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(ssid, password);
  Serial.print("WiFi AP started: ");
  Serial.println(WiFi.softAPIP());

  // WebSocket setup
  ws.onEvent(onEvent);
  server.addHandler(&ws);
  server.begin();
  Serial.println("WebSocket server started (ws://<IP>/ws)");

  // ESP-NOW setup
  if (esp_now_init() == ESP_OK) {
    esp_now_register_recv_cb(onRecv);
    Serial.println("ESP-NOW ready");
    blinkLED(3, 200);  // Blink 3 times on successful init
  } else {
    Serial.println("ESP-NOW init failed!");
    blinkLED(5, 500);  // Error blink: 5 long blinks
  }

  Serial.print("ESP MAC: ");
  Serial.println(WiFi.macAddress());
}

// ====== Main Loop ======
void loop() {
  // Nothing heavy; all handled by callbacks
  delay(10);
}