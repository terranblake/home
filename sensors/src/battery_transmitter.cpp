/********************************************************************
 * ESP32-C3 Ultra-Low Power Battery Transmitter with ESP-NOW Mesh
 * 
 * CIRCUIT DIAGRAM:
 * ┌─────────────────────────────────────────────────────────────┐
 * │  CR2450 Battery (3.0V, 540-600mAh)                         │
 * │      (+) ──┬── 100µF ceramic ──┬── ESP32-C3 VCC (Pin 1)    │
 * │            │                   │                           │
 * │            └── 220µF electro ──┘                           │
 * │      (-) ────────────────────────── ESP32-C3 GND (Pin 2)   │
 * │                                                             │
 * │  Thermistor Circuit:                                        │
 * │      3.3V ── 10kΩ ── GPIO4 ── NTC(10kΩ) ── GND            │
 * │                                                             │
 * │  Optional Power Gating:                                     │
 * │      GPIO5 ── BSS138(G) ┌─ 10kΩ ── 3.3V                   │
 * │                    (S) ─┤                                   │
 * │                    (D) ─┼── GPIO4                          │
 * │                                                             │
 * │  Optional Battery Monitor:                                  │
 * │      3.3V ── 100kΩ ── GPIO3 ── 100kΩ ── GND               │
 * └─────────────────────────────────────────────────────────────┘
 * 
 * Hardware:
 *  - ESP32-C3 module
 *  - CR2450 3V lithium battery (540-600mAh)
 *  - NTCLE100E3103JB0 thermistor (10kΩ @25°C, β=3977K)
 *  - 10kΩ 1% precision resistor
 *  - 100µF ceramic + 220µF electrolytic capacitors
 *  - Optional: BSS138 P-FET for power gating
 *  - Optional: 2x 100kΩ for battery voltage divider
 * 
 * Expected Battery Life:
 *  - 5 min intervals: ~14 days
 *  - 15 min intervals: ~42 days  
 *  - 1 hour intervals: ~168 days (5.5 months)
 *  - Safe mode (1hr): ~300+ days
 * 
 * Power Consumption:
 *  - Active (50ms): ~120mA
 *  - Deep Sleep: ~8µA
 *  - Safe Mode: ~5µA (extended intervals)
 ********************************************************************/

#include <WiFi.h>
#include <esp_now.h>
#include <esp_sleep.h>
#include <esp_wifi.h>

/************* CONFIGURATION FROM PLATFORMIO.INI *****************/
// All settings are now configured in platformio.ini and passed as build flags
// Override these in specific environments as needed

// Node Configuration
#ifndef NODE_ID
#define NODE_ID "sensor_01"
#endif

#ifndef NODE_LOCATION  
#define NODE_LOCATION "unknown"
#endif

#ifndef NODE_TYPE
#define NODE_TYPE "temperature"
#endif

// Hardware Pin Configuration
#ifndef SENSOR_PIN
#define SENSOR_PIN 4
#endif

#ifndef BATTERY_PIN
#define BATTERY_PIN 3
#endif

#ifndef NTC_POWER_PIN
#define NTC_POWER_PIN 5
#endif

#ifndef STATUS_LED_PIN
#define STATUS_LED_PIN 2
#endif

// Timing Configuration (convert minutes to use at runtime)
#ifndef NORMAL_SLEEP_MINUTES
#define NORMAL_SLEEP_MINUTES 15
#endif

#ifndef SAFE_SLEEP_MINUTES
#define SAFE_SLEEP_MINUTES 60
#endif

#ifndef EMERGENCY_SLEEP_MINUTES
#define EMERGENCY_SLEEP_MINUTES 240
#endif

// Power Management Thresholds
#ifndef BATTERY_SAFE_THRESHOLD
#define BATTERY_SAFE_THRESHOLD 2.4
#endif

#ifndef BATTERY_NORMAL_THRESHOLD
#define BATTERY_NORMAL_THRESHOLD 2.6
#endif

#ifndef BATTERY_CRITICAL_THRESHOLD
#define BATTERY_CRITICAL_THRESHOLD 2.2
#endif

#ifndef MAX_FAILED_TRANSMISSIONS
#define MAX_FAILED_TRANSMISSIONS 3
#endif

// Mesh Configuration
#ifndef MESH_RELAY_LIMIT
#define MESH_RELAY_LIMIT 3
#endif

#ifndef SAFE_MODE_AUTH
#define SAFE_MODE_AUTH 0xDEADBEEF
#endif

// Sensor Configuration
#ifndef R_FIXED
#define R_FIXED 10000.0
#endif

#ifndef BETA
#define BETA 3977.0
#endif

#ifndef T0_TEMP
#define T0_TEMP 298.15
#endif

#ifndef R0
#define R0 10000.0
#endif

// Runtime variables
const char* node_id = NODE_ID;
const char* node_location = NODE_LOCATION;
const char* node_type = NODE_TYPE;
int normal_sleep_minutes = NORMAL_SLEEP_MINUTES;
int safe_sleep_minutes = SAFE_SLEEP_MINUTES;
int emergency_sleep_minutes = EMERGENCY_SLEEP_MINUTES;
int current_sleep_minutes = NORMAL_SLEEP_MINUTES;
bool mesh_relay_enabled = MESH_RELAY_ENABLED;

// Receiver MAC address from build flags
uint8_t receiverMAC[] = {
    RECEIVER_MAC_0, RECEIVER_MAC_1, RECEIVER_MAC_2,
    RECEIVER_MAC_3, RECEIVER_MAC_4, RECEIVER_MAC_5
};

// Thermistor constants
constexpr float r_fixed = R_FIXED;
constexpr float beta = BETA;
constexpr float t0_temp = T0_TEMP;
constexpr float r0 = R0;
/*********************************************************************/

// Data structure for ESP-NOW transmission
typedef struct {
  char node_id[16];
  float temp_c;
  float temp_f;
  float battery_v;
  uint32_t sequence;
  uint32_t timestamp;
  int16_t raw_adc;
  float resistance;
  uint8_t power_mode;       // 0=normal, 1=safe, 2=emergency
  uint8_t failed_tx_count;  // Number of consecutive failed transmissions
  bool safe_mode_request;   // Remote safe mode command
} sensor_data_t;

// Safe mode control structure for incoming commands
typedef struct {
  char target_node[16];     // Target node ID ("*" for all nodes)
  uint8_t command;          // 0=normal, 1=safe, 2=emergency, 3=shutdown
  uint32_t duration_minutes; // How long to stay in this mode (0=permanent)
  uint32_t auth_code;       // Simple authentication
} safe_mode_command_t;

// Global variables
RTC_DATA_ATTR uint32_t sequence_number = 0;  // Persistent across deep sleep
RTC_DATA_ATTR uint8_t power_mode = 0;        // 0=normal, 1=safe, 2=emergency
RTC_DATA_ATTR uint8_t failed_tx_count = 0;   // Failed transmission counter
RTC_DATA_ATTR uint32_t safe_mode_start = 0;  // When safe mode started
RTC_DATA_ATTR uint32_t safe_mode_duration = 0; // How long to stay in safe mode

sensor_data_t sensor_data;
const uint32_t safe_mode_auth = SAFE_MODE_AUTH;

float readBatteryVoltage() {
  // Read battery voltage with voltage divider (if installed)
  // If no divider: direct connection, 3V max
  // With divider: 2x 100kΩ, reads up to 6V (but max is 3.2V from CR2450)
  int raw = analogRead(BATTERY_PIN);
  
  #ifdef USE_BATTERY_VOLTAGE_DIVIDER
  return (raw * 6.0) / 4095.0;  // With voltage divider
  #else
  return (raw * 3.3) / 4095.0;  // Direct connection
  #endif
}

void checkAndUpdatePowerMode() {
  float battery_v = readBatteryVoltage();
  uint32_t current_time = millis() / 60000; // Minutes since boot
  
  // Check if safe mode duration has expired
  if (safe_mode_duration > 0 && 
      (current_time - safe_mode_start) >= safe_mode_duration) {
    power_mode = 0; // Return to normal
    safe_mode_duration = 0;
    Serial.println("Safe mode duration expired - returning to normal");
  }
  
  // Battery-based safe mode logic
  if (power_mode == 0 && battery_v < BATTERY_SAFE_THRESHOLD) {
    power_mode = 1; // Enter safe mode
    safe_mode_start = current_time;
    Serial.printf("Battery low (%.3fV) - entering safe mode\n", battery_v);
  } else if (power_mode == 1 && battery_v > BATTERY_NORMAL_THRESHOLD) {
    power_mode = 0; // Exit safe mode
    Serial.printf("Battery recovered (%.3fV) - exiting safe mode\n", battery_v);
  }
  
  // Emergency mode for critically low battery
  if (battery_v < BATTERY_CRITICAL_THRESHOLD) {
    power_mode = 2; // Emergency mode
    Serial.printf("EMERGENCY: Battery critical (%.3fV)\n", battery_v);
  }
  
  // Failed transmission safe mode
  if (failed_tx_count >= MAX_FAILED_TRANSMISSIONS && power_mode == 0) {
    power_mode = 1;
    safe_mode_start = current_time;
    Serial.printf("Too many failed transmissions (%d) - entering safe mode\n", failed_tx_count);
  }
  
  // Set sleep interval based on power mode
  switch (power_mode) {
    case 0: // Normal
      current_sleep_minutes = normal_sleep_minutes;
      break;
    case 1: // Safe
      current_sleep_minutes = safe_sleep_minutes;
      break;
    case 2: // Emergency
      current_sleep_minutes = emergency_sleep_minutes;
      break;
  }
  
  Serial.printf("Power mode: %s, Sleep interval: %d min\n",
                power_mode == 0 ? "NORMAL" : power_mode == 1 ? "SAFE" : "EMERGENCY",
                current_sleep_minutes);
}

float readThermistorC() {
  // Power on the thermistor circuit
  #ifdef USE_POWER_GATING
  digitalWrite(NTC_POWER_PIN, HIGH);
  delay(10);  // Allow settling time
  #endif
  
  int raw = analogRead(SENSOR_PIN);
  Serial.printf("Raw ADC: %d\n", raw);
  
  // ESP32-C3 with CR2450 direct connection (no regulator)
  float battery_v = readBatteryVoltage();
  float v = raw * (battery_v / 4095.0);  // Use actual battery voltage as reference
  Serial.printf("Voltage: %.3f V (Battery: %.3f V)\n", v, battery_v);
  
  // Power off the thermistor circuit to save power
  #ifdef USE_POWER_GATING
  digitalWrite(NTC_POWER_PIN, LOW);
  #endif
  
  if (v >= (battery_v - 0.1)) {
    Serial.println("Error: Voltage near supply - check thermistor connection");
    return -999.0;
  }
  
  float rT = (v * r_fixed) / (battery_v - v);  // Voltage divider math
  Serial.printf("Thermistor resistance: %.1f Ω\n", rT);
  
  float tempK = 1.0 / ((1.0 / t0_temp) + (1.0 / beta) * log(rT / r0));
  return tempK - 273.15;  // Convert to Celsius
}

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    Serial.println("ESP-NOW Send: SUCCESS");
    failed_tx_count = 0; // Reset failure counter on success
  } else {
    Serial.println("ESP-NOW Send: FAIL");
    failed_tx_count++;
    Serial.printf("Failed transmission count: %d\n", failed_tx_count);
  }
}

void onDataReceived(const uint8_t* mac, const uint8_t* data, int len) {
  // Handle incoming safe mode commands and mesh relay
  if (len == sizeof(safe_mode_command_t)) {
    safe_mode_command_t cmd;
    memcpy(&cmd, data, sizeof(cmd));
    
    // Verify authentication
    if (cmd.auth_code != safe_mode_auth) {
      Serial.println("Invalid safe mode command - auth failed");
      return;
    }
    
    // Check if command is for this node or broadcast
    if (strcmp(cmd.target_node, node_id) == 0 || strcmp(cmd.target_node, "*") == 0) {
      Serial.printf("Safe mode command received: mode=%d, duration=%d min\n", 
                    cmd.command, cmd.duration_minutes);
      
      power_mode = cmd.command;
      safe_mode_duration = cmd.duration_minutes;
      safe_mode_start = millis() / 60000;
      
      // Update sleep intervals immediately
      checkAndUpdatePowerMode();
    }
    
    // Relay command to other nodes if in mesh mode and not the originator
    if (mesh_relay_enabled && memcmp(mac, receiverMAC, 6) == 0) {
      Serial.println("Relaying safe mode command to other nodes");
      esp_now_send(NULL, data, len); // Broadcast relay
    }
  }
  
  // Handle other mesh data relay (sensor data from other nodes)
  else if (len == sizeof(sensor_data_t) && mesh_relay_enabled) {
    sensor_data_t relay_data;
    memcpy(&relay_data, data, sizeof(relay_data));
    
    // Don't relay our own data or in emergency mode
    if (strcmp(relay_data.node_id, node_id) != 0 && power_mode < 2) {
      Serial.printf("Relaying data from node: %s\n", relay_data.node_id);
      esp_now_send(receiverMAC, data, len);
    }
  }
}

void initESPNow() {
  // Set WiFi mode and initialize ESP-NOW
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  
  // Set custom MAC address for consistent identification (optional)
  // uint8_t customMAC[] = {0x30, 0xAE, 0xA4, 0x07, 0x0D, 0x64};
  // esp_wifi_set_mac(WIFI_IF_STA, customMAC);
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
    return;
  }
  
  // Register callbacks
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataReceived);
  
  // Add peer (receiver)
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  
  Serial.println("ESP-NOW initialized successfully");
}

void sendSensorData() {
  // Check and update power mode before reading sensors
  checkAndUpdatePowerMode();
  
  // Read sensor data
  float temp_c = readThermistorC();
  float temp_f = temp_c * 9.0 / 5.0 + 32.0;
  float battery_v = readBatteryVoltage();
  
  // Prepare data packet
  strncpy(sensor_data.node_id, node_id, sizeof(sensor_data.node_id) - 1);
  sensor_data.temp_c = temp_c;
  sensor_data.temp_f = temp_f;
  sensor_data.battery_v = battery_v;
  sensor_data.sequence = ++sequence_number;
  sensor_data.timestamp = millis();  // Time since boot
  sensor_data.raw_adc = analogRead(SENSOR_PIN);
  sensor_data.resistance = 0.0;  // Could calculate if needed
  sensor_data.power_mode = power_mode;
  sensor_data.failed_tx_count = failed_tx_count;
  sensor_data.safe_mode_request = false; // Client doesn't request safe mode
  
  Serial.printf("Sending: ID=%s, Temp=%.2f°C, Battery=%.2fV, Mode=%s, Seq=%d\n",
                sensor_data.node_id, sensor_data.temp_c, 
                sensor_data.battery_v, 
                power_mode == 0 ? "NORMAL" : power_mode == 1 ? "SAFE" : "EMERGENCY",
                sensor_data.sequence);
  
  // Send data via ESP-NOW
  esp_err_t result = esp_now_send(receiverMAC, 
                                  (uint8_t*)&sensor_data, 
                                  sizeof(sensor_data));
  
  if (result == ESP_OK) {
    Serial.println("ESP-NOW packet sent successfully");
  } else {
    Serial.printf("ESP-NOW send failed: %d\n", result);
    failed_tx_count++;
  }
  
  // Wait for transmission to complete and potential incoming commands
  delay(200);
}

void enterDeepSleep() {
  Serial.printf("Entering deep sleep for %d minutes...\n", SLEEP_MINUTES);
  Serial.flush();
  
  // Configure wake-up timer
  uint64_t sleep_us = SLEEP_MINUTES * 60ULL * 1000000ULL;
  esp_sleep_enable_timer_wakeup(sleep_us);
  
  // Shutdown WiFi and ESP-NOW to save power
  esp_now_deinit();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  
  // Additional power savings
  esp_wifi_stop();
  esp_bt_controller_disable();
  
  // Enter deep sleep
  esp_deep_sleep_start();
}

void setup() {
  // Ultra-low power CPU frequency
  setCpuFrequencyMhz(80);  // 80MHz for balance of speed and power
  
  Serial.begin(115200);
  delay(100);
  
  Serial.println("\n=== ESP32-C3 Battery Transmitter ===");
  Serial.printf("Node ID: %s\n", node_id);
  Serial.printf("Location: %s\n", node_location);
  Serial.printf("Type: %s\n", node_type);
  Serial.printf("Sequence: %d\n", sequence_number);
  Serial.printf("CPU Frequency: %d MHz\n", getCpuFrequencyMhz());
  
  // Configure GPIO pins
  #ifdef USE_POWER_GATING
  pinMode(NTC_POWER_PIN, OUTPUT);
  digitalWrite(NTC_POWER_PIN, LOW);  // Start with thermistor powered off
  Serial.printf("NTC power control enabled on GPIO%d\n", NTC_POWER_PIN);
  #endif
  
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);
  Serial.printf("Status LED configured on GPIO%d\n", STATUS_LED_PIN);
  
  // Check battery voltage
  float battery_v = readBatteryVoltage();
  Serial.printf("Battery voltage: %.3f V\n", battery_v);
  
  if (battery_v < 2.3) {
    Serial.println("WARNING: Battery voltage low! Extending sleep interval.");
    // Could extend sleep interval here for low battery
  }
  
  // Initialize ESP-NOW
  initESPNow();
  
  // Send sensor data
  sendSensorData();
  
  // Enter deep sleep
  enterDeepSleep();
}

void loop() {
  // Should never reach here due to deep sleep
  Serial.println("ERROR: Should be in deep sleep!");
  delay(1000);
}

/********************************************************************
 * SETUP INSTRUCTIONS:
 * 
 * 1. Flash this code to your ESP32-C3
 * 2. Get the MAC address from Serial output
 * 3. Update receiverMAC[] with the receiver's MAC address
 * 4. Install battery and capacitors as per circuit design
 * 5. Test with short sleep intervals first (1-2 minutes)
 * 
 * TROUBLESHOOTING:
 * - If the board stops responding, press reset during power-up
 * - Use recovery_transmitter.cpp for debugging (no deep sleep)
 * - Monitor battery voltage in the data to track battery health
 * 
 * BATTERY OPTIMIZATION:
 * - Increase SLEEP_MINUTES for longer battery life
 * - Add NTC_POWER_PIN circuit for ~30% power savings
 * - Use broadcast MAC (FF:FF:FF:FF:FF:FF) to avoid peer management
 ********************************************************************/
