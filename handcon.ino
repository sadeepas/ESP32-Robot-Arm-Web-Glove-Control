/*
   HandController.ino
   ESP32 DevKit V1 (glove) - reads one flex sensor and sends ESP-NOW commands
   to the Robot Arm (struct with base, shoulder, elbow, gripper).
   Compatible with the provided Robot Arm code.
   Required libraries: ESP32 core (includes ESP-NOW).
   Hardware: One flex sensor on pin 34 (voltage divider).
   Tune calibration values using Serial debug.
   - LED on GPIO 2: Blinks on ESP-NOW init success (3x), and on each send (1x short blink)
   - Configured for unicast to robot arm MAC (example provided; replace with actual).
*/

// ====== Includes ======
#include <WiFi.h>
#include <esp_now.h>

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

// ====== ESP-NOW Command Structure (must match robot arm) ======
typedef struct {
  uint16_t seq;
  uint8_t base;
  uint8_t shoulder;
  uint8_t elbow;
  uint8_t gripper;
  uint8_t battery;
} __attribute__((packed)) ArmCommand;
ArmCommand cmd;

// ====== Flex Sensor Setup (for Gripper only) ======
#define GRIPPER_FLEX_PIN 34
#define GRIPPER_FLEX_STRAIGHT 1700
#define GRIPPER_FLEX_BENT 3100
#define GRIPPER_SERVO_OPEN 120 // Map to arm's 0-180 scale
#define GRIPPER_SERVO_CLOSED 40

// ====== Helper Functions ======
int mapAndClamp(int x, int in_min, int in_max, int out_min, int out_max) {
  int result = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  if (result > out_max) return out_max;
  if (result < out_min) return out_min;
  return result;
}

uint8_t robotMac[] = {0x24, 0x6F, 0x28, 0xAB, 0xCD, 0xEF}; // EXAMPLE MAC: Replace with actual robot arm MAC (e.g., from Serial output: 24:6F:28:AB:CD:EF)

void sendCommand() {
  // Set default positions for other joints (90° neutral)
  cmd.base = 90;
  cmd.shoulder = 90;
  cmd.elbow = 90;
  // Map flex to gripper (straight = open, bent = closed)
  int flexValue = analogRead(GRIPPER_FLEX_PIN);
  cmd.gripper = mapAndClamp(flexValue, GRIPPER_FLEX_STRAIGHT, GRIPPER_FLEX_BENT, GRIPPER_SERVO_OPEN, GRIPPER_SERVO_CLOSED);
  cmd.battery = 100; // Dummy battery level
  cmd.seq++; // Increment sequence
  // Unicast send to robot MAC
  esp_err_t result = esp_now_send(robotMac, (uint8_t*)&cmd, sizeof(cmd));
  if (result == ESP_OK) {
    Serial.println("Send OK");
  } else {
    Serial.printf("Send Error: %d\n", result);
  }
 
  // Blink LED on send
  blinkLED(1, 100);
 
  Serial.printf("Sent: seq=%d B=%d S=%d E=%d G=%d (raw %4d)\n",
                cmd.seq, cmd.base, cmd.shoulder, cmd.elbow, cmd.gripper, flexValue);
}

void setup() {
  Serial.begin(115200);
  delay(20);
  Serial.println("\nStarting ESP32 DevKit V1 Hand Controller (Single Flex for Gripper)...");
  // Initialize LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  // Initialize WiFi and ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("FATAL: ESP-NOW init failed.");
    blinkLED(5, 500); // Error blink: 5 long blinks
    while (1) delay(10);
  }
  // Add peer for unicast (robot arm MAC)
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, robotMac, 6);
  peerInfo.channel = 0; // 0 = current channel (auto)
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer!");
    blinkLED(5, 200); // Error blink
    while (1) delay(10);
  }
  Serial.println("Peer added successfully.");
  Serial.print("ESP MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.printf("Target Robot MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                robotMac[0], robotMac[1], robotMac[2], robotMac[3], robotMac[4], robotMac[5]);
  Serial.println("ESP-NOW initialized (unicast mode).");
  blinkLED(3, 200); // Blink 3 times on successful init
  // Initial command (neutral position)
  cmd.seq = 0;
  sendCommand();
}

void loop() {
  sendCommand();
  delay(50); // ~20Hz update rate
}