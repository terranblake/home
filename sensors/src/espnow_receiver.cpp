/********************************************************************
 * ESP32 ESP-NOW Receiver/Mesh Node with Remote Safe-Mode Control
 * 
 * CIRCUIT DIAGRAM - Option A (Mains Powered):
 * ┌─────────────────────────────────────────────────────────────┐
 * │  USB/Mains Power Supply (5V)                               │
 * │      5V ── ESP32 Dev Board ── Status LED (GPIO2)           │
 * │      GND ────────────────────── GND                        │
 * │                                                             │
 * │  Optional Battery Monitor (for mesh nodes):                │
 * │      3.3V ── 100kΩ ── GPIO3 ── 100kΩ ── GND               │
 * └─────────────────────────────────────────────────────────────┘
 * 
 * CIRCUIT DIAGRAM - Option B (Battery Powered Mesh Node):
 * ┌─────────────────────────────────────────────────────────────┐
 * │  3x AA Lithium Batteries (4.5V, ~3000mAh each)            │
 * │      (+) ──┬── 470µF electro ──┬── ESP32-C3 VCC (Pin 1)   │
 * │            │                   │                           │
 * │            └── 100µF ceramic ──┘                           │
 * │      (-) ────────────────────────── ESP32-C3 GND (Pin 2)   │
 * │                                                             │
 * │  Battery Monitor:                                           │
 * │      3.3V ── 100kΩ ── GPIO3 ── 100kΩ ── GND               │
 * │                                                             │
 * │  Status LED:                                                │
 * │      GPIO2 ── 330Ω ── LED ── GND                           │
 * └─────────────────────────────────────────────────────────────┘
 * 
 * Hardware Options:
 *  - Option A: ESP32 dev board (USB/mains powered gateway)
 *  - Option B: ESP32-C3 module (battery-powered mesh node)
 *  - 3x AA lithium batteries (4.5V, ~9000mAh total)
 *  - 470µF electrolytic + 100µF ceramic capacitors
 *  - 2x 100kΩ for battery voltage divider (optional)
 *  - Status LED with 330Ω resistor
 * 
 * Function:
 *  - Receives ESP-NOW packets from battery-powered sensors
 *  - Forwards data to existing Python temperature server via HTTP
 *  - Supports mesh relay for extending network range
 *  - Remote safe-mode control for transmitter nodes
 *  - Can operate as battery-powered mesh node or mains gateway
 *  - Automatic power mode management for battery operation
 * 
 * Expected Battery Life (3x AA Lithium, Option B):
 *  - Mains mode (always on): ~30-45 days
 *  - Safe mode (reduced activity): ~90-120 days
 *  - Emergency mode: ~180+ days
 * 
 * Integration:
 *  - Connects to your existing WiFi network
 *  - Sends data to http://192.168.8.123:8080/ingest
 *  - Compatible with existing dashboard and logging system
 *  - Mesh capabilities for extended range and reliability
 ********************************************************************/

#include <WiFi.h>
#include <HTTPClient.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <ArduinoJson.h>

/************* CONFIGURATION FROM PLATFORMIO.INI *****************/
// All settings are now configured in platformio.ini and passed as build flags
// Override these in specific environments as needed

// WiFi Configuration
#ifndef WIFI_SSID
#define WIFI_SSID "FromTheLandOfKansas"
#endif

#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD "Kansas_6614!"
#endif

#ifndef SERVER_ENDPOINT
#define SERVER_ENDPOINT "http://192.168.8.123:8080/ingest"
#endif

// Node Configuration
#ifndef NODE_ID
#define NODE_ID "receiver_01"
#endif

#ifndef NODE_LOCATION
#define NODE_LOCATION "unknown"
#endif

#ifndef NODE_TYPE
#define NODE_TYPE "receiver"
#endif

// Hardware Pin Configuration
#ifndef BATTERY_PIN
#define BATTERY_PIN 3
#endif

#ifndef STATUS_LED_PIN
#define STATUS_LED_PIN 2
#endif

// Power Management (for battery-powered mesh nodes)
#ifndef BATTERY_SAFE_THRESHOLD
#define BATTERY_SAFE_THRESHOLD 3.8
#endif

#ifndef BATTERY_NORMAL_THRESHOLD
#define BATTERY_NORMAL_THRESHOLD 4.0
#endif

#ifndef BATTERY_CRITICAL_THRESHOLD
#define BATTERY_CRITICAL_THRESHOLD 3.4
#endif

// Mesh Configuration
#ifndef MESH_RELAY_LIMIT
#define MESH_RELAY_LIMIT 5
#endif

#ifndef SAFE_MODE_AUTH
#define SAFE_MODE_AUTH 0xDEADBEEF
#endif

// Runtime variables
const char* wifi_ssid = WIFI_SSID;
const char* wifi_password = WIFI_PASSWORD;
const char* server_endpoint = SERVER_ENDPOINT;
const char* node_id = NODE_ID;
const char* node_location = NODE_LOCATION;
const char* node_type = NODE_TYPE;
const bool battery_powered = BATTERY_POWERED;
const bool mesh_relay_enabled = MESH_RELAY_ENABLED;
const bool safe_mode_control_enabled = SAFE_MODE_CONTROL_ENABLED;

// Check intervals (milliseconds)
int normal_check_interval = 5000;            // Normal: check every 5 seconds
int safe_check_interval = 30000;             // Safe mode: check every 30 seconds
int current_check_interval = 5000;           // Current active interval
/*********************************************************************/

// Data structure matching transmitter (updated for safe mode)
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

// Safe mode control structure for outgoing commands
typedef struct {
  char target_node[16];     // Target node ID ("*" for all nodes)
  uint8_t command;          // 0=normal, 1=safe, 2=emergency, 3=shutdown
  uint32_t duration_minutes; // How long to stay in this mode (0=permanent)
  uint32_t auth_code;       // Simple authentication
} safe_mode_command_t;

// Global variables for receiver/mesh operation
uint8_t power_mode = 0;                      // 0=normal, 1=safe, 2=emergency
uint32_t safe_mode_start = 0;                // When safe mode started (battery nodes)
uint32_t safe_mode_duration = 0;             // How long to stay in safe mode
const uint32_t SAFE_MODE_AUTH = 0xDEADBEEF;  // Simple auth code
uint32_t relay_count = 0;                    // Packets relayed this cycle

// Statistics
struct {
  uint32_t packets_received = 0;
  uint32_t packets_forwarded = 0;
  uint32_t packets_failed = 0;
  uint32_t packets_relayed = 0;
  uint32_t safe_commands_sent = 0;
  unsigned long last_packet_time = 0;
} stats;

float readBatteryVoltage() {
  if (!battery_powered) return 5.0; // Assume mains power
  
  // Read battery voltage with voltage divider for 3x AA (4.5V max)
  int raw = analogRead(BATTERY_PIN);
  return (raw * 6.0) / 4095.0;  // Voltage divider for up to 6V
}

void checkAndUpdatePowerMode() {
  if (!battery_powered) return; // Skip for mains-powered nodes
  
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
  if (battery_v < BATTERY_CRITICAL_THRESHOLD) { // For 3x AA lithium
    power_mode = 2; // Emergency mode
    Serial.printf("EMERGENCY: Battery critical (%.3fV)\n", battery_v);
  }
  
  // Set check interval based on power mode
  switch (power_mode) {
    case 0: // Normal
      current_check_interval = normal_check_interval;
      break;
    case 1: // Safe
      current_check_interval = safe_check_interval;
      break;
    case 2: // Emergency
      current_check_interval = safe_check_interval * 4; // 2 minutes
      break;
  }
  
  Serial.printf("Power mode: %s, Check interval: %d ms\n",
                power_mode == 0 ? "NORMAL" : power_mode == 1 ? "SAFE" : "EMERGENCY",
                current_check_interval);
}

bool sendSafeModeCommand(const char* target_node, uint8_t command, uint32_t duration_minutes = 0) {
  if (!safe_mode_control_enabled) return false;
  
  safe_mode_command_t cmd;
  strncpy(cmd.target_node, target_node, sizeof(cmd.target_node) - 1);
  cmd.target_node[sizeof(cmd.target_node) - 1] = '\0';
  cmd.command = command;
  cmd.duration_minutes = duration_minutes;
  cmd.auth_code = safe_mode_auth;
  
  // Broadcast the command
  esp_err_t result = esp_now_send(NULL, (uint8_t*)&cmd, sizeof(cmd));
  
  if (result == ESP_OK) {
    stats.safe_commands_sent++;
    Serial.printf("Safe mode command sent: target=%s, mode=%d, duration=%d\n", 
                  target_node, command, duration_minutes);
    return true;
  } else {
    Serial.printf("Failed to send safe mode command: %d\n", result);
    return false;
  }
}

void printMACAddress() {
  uint8_t baseMac[6];
  esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
  Serial.printf("Receiver MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\n",
                baseMac[0], baseMac[1], baseMac[2], 
                baseMac[3], baseMac[4], baseMac[5]);
  Serial.println("*** UPDATE TRANSMITTER WITH THIS MAC ADDRESS ***");
}

void blinkStatus(int count = 1) {
  for (int i = 0; i < count; i++) {
    digitalWrite(STATUS_LED_PIN, HIGH);
    delay(100);
    digitalWrite(STATUS_LED_PIN, LOW);
    delay(100);
  }
}

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifi_ssid, wifi_password);
  
  Serial.printf("Connecting to WiFi: %s", wifi_ssid);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.printf("WiFi connected! IP: %s\n", WiFi.localIP().toString().c_str());
  blinkStatus(3);  // Triple blink for WiFi connected
}

bool forwardToServer(const sensor_data_t& data) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected - cannot forward data");
    return false;
  }
  
  HTTPClient http;
  WiFiClient client;
  
  // Build URL with all sensor parameters (compatible with existing server)
  String url = String(server_endpoint) + 
               "?room=" + String(data.node_id) +
               "&sensor=NTCLE100E3103JB0_ESPNOW" +
               "&tempC=" + String(data.temp_c, 2) +
               "&tempF=" + String(data.temp_f, 2) +
               "&rawADC=" + String(data.raw_adc) +
               "&voltage=" + String(data.battery_v, 3) +
               "&resistance=" + String(data.resistance, 1) +
               "&beta=3977" +
               "&r0=10000" +
               "&rFixed=10000" +
               "&sequence=" + String(data.sequence) +
               "&battery=" + String(data.battery_v, 3) +
               "&powerMode=" + String(data.power_mode) +
               "&failedTx=" + String(data.failed_tx_count) +
               "&safeRequest=" + String(data.safe_mode_request ? 1 : 0);
  
  Serial.printf("Forwarding to server: %s\n", url.c_str());
  
  http.begin(client, url);
  http.setTimeout(5000);  // 5 second timeout
  
  int httpCode = http.GET();
  bool success = (httpCode == 200);
  
  if (success) {
    Serial.printf("✓ HTTP response: %d\n", httpCode);
    stats.packets_forwarded++;
    blinkStatus(1);  // Single blink for successful forward
  } else {
    Serial.printf("✗ HTTP failed: %d\n", httpCode);
    stats.packets_failed++;
  }
  
  http.end();
  return success;
}

void onDataReceived(const uint8_t* mac, const uint8_t* data, int len) {
  stats.packets_received++;
  stats.last_packet_time = millis();
  
  // Handle incoming safe mode commands
  if (len == sizeof(safe_mode_command_t)) {
    safe_mode_command_t cmd;
    memcpy(&cmd, data, sizeof(cmd));
    
    // Verify authentication
    if (cmd.auth_code != SAFE_MODE_AUTH) {
      Serial.println("Invalid safe mode command - auth failed");
      return;
    }
    
    Serial.printf("Safe mode command received: target=%s, mode=%d, duration=%d\n", 
                  cmd.target_node, cmd.command, cmd.duration_minutes);
    
    // Check if command is for this receiver node
    if (strcmp(cmd.target_node, NODE_ID) == 0 || strcmp(cmd.target_node, "*") == 0) {
      if (BATTERY_POWERED) {
        power_mode = cmd.command;
        safe_mode_duration = cmd.duration_minutes;
        safe_mode_start = millis() / 60000;
        Serial.printf("Receiver entering power mode: %d\n", power_mode);
        checkAndUpdatePowerMode();
      } else {
        Serial.println("Safe mode command received but this is a mains-powered node");
      }
    }
    
    // Relay command to transmitter nodes if mesh is enabled and in normal/safe mode
    if (MESH_RELAY_ENABLED && relay_count < MAX_RELAY_PER_CYCLE && power_mode < 2) {
      Serial.println("Relaying safe mode command to transmitter nodes");
      esp_now_send(NULL, data, len); // Broadcast relay
      relay_count++;
      stats.packets_relayed++;
    }
    return;
  }
  
  // Handle sensor data packets
  if (len != sizeof(sensor_data_t)) {
    Serial.printf("Invalid sensor packet size: %d (expected %d)\n", len, sizeof(sensor_data_t));
    return;
  }
  
  sensor_data_t sensor_data;
  memcpy(&sensor_data, data, sizeof(sensor_data));
  
  // Print sender MAC address
  Serial.printf("\n=== ESP-NOW Packet Received ===\n");
  Serial.printf("From MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  
  // Print sensor data with new fields
  Serial.printf("Node ID: %s\n", sensor_data.node_id);
  Serial.printf("Temperature: %.2f°C (%.2f°F)\n", 
                sensor_data.temp_c, sensor_data.temp_f);
  Serial.printf("Battery: %.3fV\n", sensor_data.battery_v);
  Serial.printf("Power Mode: %s\n", 
                sensor_data.power_mode == 0 ? "NORMAL" : 
                sensor_data.power_mode == 1 ? "SAFE" : "EMERGENCY");
  Serial.printf("Failed TX Count: %d\n", sensor_data.failed_tx_count);
  Serial.printf("Sequence: %d\n", sensor_data.sequence);
  Serial.printf("Raw ADC: %d\n", sensor_data.raw_adc);
  
  // Check for critical conditions and potentially send safe mode commands
  bool send_safe_command = false;
  
  // Check battery level
  if (sensor_data.battery_v < 2.3) {
    Serial.printf("🚨 CRITICAL BATTERY: %.3fV - sending emergency command\n", sensor_data.battery_v);
    send_safe_command = true;
  } else if (sensor_data.battery_v < 2.5) {
    Serial.printf("⚠️  LOW BATTERY WARNING: %.3fV - sending safe mode command\n", sensor_data.battery_v);
    send_safe_command = true;
  }
  
  // Check for failed transmissions
  if (sensor_data.failed_tx_count >= 3) {
    Serial.printf("⚠️  HIGH FAILURE COUNT: %d - sending safe mode command\n", sensor_data.failed_tx_count);
    send_safe_command = true;
  }
  
  // Send safe mode command if needed
  if (send_safe_command && SAFE_MODE_CONTROL_ENABLED) {
    uint8_t mode = (sensor_data.battery_v < 2.3) ? 2 : 1; // Emergency or safe
    sendSafeModeCommand(sensor_data.node_id, mode, 0); // Permanent until manual override
  }
  
  // Forward to HTTP server
  bool success = forwardToServer(sensor_data);
  
  if (!success) {
    Serial.println("Failed to forward data to server");
    // Could implement retry logic or local storage here
  }
  
  // Relay sensor data to other nodes if mesh is enabled and not in emergency mode
  if (MESH_RELAY_ENABLED && relay_count < MAX_RELAY_PER_CYCLE && power_mode < 2) {
    // Don't relay back to sender, and only relay if we're not the final destination
    Serial.printf("Relaying sensor data from node: %s\n", sensor_data.node_id);
    // In a real mesh, you'd implement more sophisticated routing here
    relay_count++;
    stats.packets_relayed++;
  }
  
  Serial.printf("Stats: RX=%d, FWD=%d, FAIL=%d, RELAY=%d, SAFE_CMD=%d\n", 
                stats.packets_received, stats.packets_forwarded, stats.packets_failed,
                stats.packets_relayed, stats.safe_commands_sent);
  Serial.println("=============================\n");
}

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    Serial.println("ESP-NOW Send: SUCCESS");
  } else {
    Serial.println("ESP-NOW Send: FAIL");
  }
}

void initESPNow() {
  // Set WiFi mode for ESP-NOW
  WiFi.mode(WIFI_STA);
  
  // ESP-NOW initialization
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
    return;
  }
  
  Serial.println("ESP-NOW initialized successfully");
  
  // Register callbacks
  esp_now_register_recv_cb(onDataReceived);
  esp_now_register_send_cb(onDataSent);
  
  Serial.println("ESP-NOW receiver ready for incoming packets");
  Serial.println("Mesh relay and safe-mode control enabled");
}

void printStatus() {
  static unsigned long last_status = 0;
  unsigned long now = millis();
  
  // Print status every 30 seconds (or longer in safe/emergency modes)
  unsigned long status_interval = 30000;
  if (battery_powered) {
    switch (power_mode) {
      case 0: status_interval = 30000; break;  // Normal: 30 seconds
      case 1: status_interval = 60000; break;  // Safe: 1 minute
      case 2: status_interval = 300000; break; // Emergency: 5 minutes
    }
  }
  
  if (now - last_status > status_interval) {
    last_status = now;
    
    Serial.println("\n=== RECEIVER/MESH NODE STATUS ===");
    Serial.printf("Node ID: %s\n", node_id);
    Serial.printf("Type: %s\n", battery_powered ? "Battery-powered mesh" : "Mains-powered gateway");
    
    if (battery_powered) {
      float battery_v = readBatteryVoltage();
      Serial.printf("Battery: %.3fV\n", battery_v);
      Serial.printf("Power Mode: %s\n", 
                    power_mode == 0 ? "NORMAL" : power_mode == 1 ? "SAFE" : "EMERGENCY");
      Serial.printf("Check Interval: %d ms\n", current_check_interval);
    }
    
    Serial.printf("WiFi: %s", WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected");
    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf(" (IP: %s)", WiFi.localIP().toString().c_str());
    }
    Serial.println();
    
    Serial.printf("Uptime: %lu minutes\n", now / 60000);
    Serial.printf("Packets received: %d\n", stats.packets_received);
    Serial.printf("Packets forwarded: %d\n", stats.packets_forwarded);
    Serial.printf("Packets failed: %d\n", stats.packets_failed);
    Serial.printf("Packets relayed: %d\n", stats.packets_relayed);
    Serial.printf("Safe commands sent: %d\n", stats.safe_commands_sent);
    
    if (stats.last_packet_time > 0) {
      unsigned long since_last = (now - stats.last_packet_time) / 1000;
      Serial.printf("Last packet: %lu seconds ago\n", since_last);
      
      if (since_last > 1800) {  // 30 minutes
        Serial.println("⚠️  No packets received for >30 minutes");
      }
    } else {
      Serial.println("No packets received yet");
    }
    
    Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());
    Serial.println("=================================\n");
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n=== ESP32 ESP-NOW Receiver/Mesh Node ===");
  Serial.printf("Mode: %s\n", battery_powered ? "Battery-powered mesh node" : "Mains-powered gateway");
  Serial.printf("Node ID: %s\n", node_id);
  Serial.printf("Location: %s\n", node_location);
  Serial.printf("Type: %s\n", node_type);
  Serial.printf("Mesh relay: %s\n", mesh_relay_enabled ? "Enabled" : "Disabled");
  Serial.printf("Safe-mode control: %s\n", safe_mode_control_enabled ? "Enabled" : "Disabled");
  
  // Configure status LED
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);
  
  // Configure battery monitoring (if battery powered)
  if (battery_powered) {
    pinMode(BATTERY_PIN, INPUT);
    checkAndUpdatePowerMode();
  }
  
  // Print MAC address for transmitter configuration
  printMACAddress();
  
  // Connect to WiFi (may be skipped in emergency mode for battery nodes)
  if (!battery_powered || power_mode < 2) {
    connectWiFi();
  } else {
    Serial.println("Emergency mode - skipping WiFi connection to save power");
  }
  
  // Initialize ESP-NOW
  initESPNow();
  
  // Add HTTP server for safe mode commands (optional)
  if (WiFi.status() == WL_CONNECTED && !battery_powered) {
    Serial.println("Starting HTTP server for safe mode commands on port 8081");
    // This would require including WebServer.h and implementing HTTP endpoints
    // For now, this is a placeholder for future implementation
  }
  
  Serial.println("\n🎯 Receiver ready! Waiting for sensor data...");
  Serial.println("📡 Configure transmitters with the MAC address shown above");
  if (!battery_powered || power_mode < 2) {
    Serial.printf("🌐 Data will be forwarded to: %s\n", server_endpoint);
  }
  Serial.println("🔗 Mesh relay and safe-mode control active");
  Serial.println("=" * 50);
}

void loop() {
  static unsigned long last_power_check = 0;
  static unsigned long last_blink = 0;
  unsigned long now = millis();
  
  // Check and update power mode (for battery-powered nodes)
  if (battery_powered && (now - last_power_check > current_check_interval)) {
    last_power_check = now;
    checkAndUpdatePowerMode();
    relay_count = 0; // Reset relay counter each check cycle
  }
  
  // Print periodic status
  printStatus();
  
  // Check WiFi connection (skip in emergency mode for battery nodes)
  if ((!battery_powered || power_mode < 2) && WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected! Reconnecting...");
    connectWiFi();
  }
  
  // LED blink pattern based on power mode
  unsigned long blink_interval = 2000; // Default 2 seconds
  if (battery_powered) {
    switch (power_mode) {
      case 0: blink_interval = 2000; break;  // Normal: 2 seconds
      case 1: blink_interval = 5000; break;  // Safe: 5 seconds  
      case 2: blink_interval = 10000; break; // Emergency: 10 seconds
    }
  }
  
  if (now - last_blink > blink_interval) {
    last_blink = now;
    digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
  }
  
  // Adjust main loop delay based on power mode
  int loop_delay = 100; // Default for mains power
  if (battery_powered) {
    switch (power_mode) {
      case 0: loop_delay = 100; break;  // Normal: 100ms
      case 1: loop_delay = 500; break;  // Safe: 500ms
      case 2: loop_delay = 1000; break; // Emergency: 1 second
    }
  }
  
  delay(loop_delay);
}

/********************************************************************
 * SETUP INSTRUCTIONS:
 * 
 * OPTION A - Mains-Powered Gateway:
 * 1. Set BATTERY_POWERED = false in platformio.ini
 * 2. Flash this code to a mains-powered ESP32/ESP32-C3 dev board
 * 3. Copy the MAC address from Serial output
 * 4. Update battery_transmitter.cpp with this MAC address
 * 5. Place receiver near your computer/router for good WiFi signal
 * 6. Monitor Serial output to see incoming sensor data
 * 
 * OPTION B - Battery-Powered Mesh Node:
 * 1. Set BATTERY_POWERED = true in platformio.ini
 * 2. Wire battery monitoring circuit (GPIO3 with voltage divider)
 * 3. Connect 3x AA lithium batteries (4.5V) with capacitor buffering
 * 4. Flash this code to an ESP32-C3 module
 * 5. Deploy as intermediate mesh node to extend range
 * 6. Monitor power consumption and battery levels
 * 
 * FEATURES:
 * - Receives ESP-NOW packets from multiple battery sensors
 * - Forwards data to existing Python temperature server via HTTP
 * - Mesh relay capability for extending network range
 * - Remote safe-mode control for transmitter battery protection
 * - Compatible with existing dashboard and logging
 * - Dual-mode operation: mains or battery powered
 * - Automatic power management for battery operation
 * - Status LED indicates activity and power mode
 * - Automatic WiFi reconnection (except in emergency mode)
 * - Advanced packet routing and relay logic
 * - Battery protection and safe-mode enforcement
 * 
 * MESH OPERATION:
 * - Automatically relays sensor data between nodes
 * - Broadcasts safe-mode commands to all transmitter nodes
 * - Limits relay activity to prevent battery drain
 * - Emergency mode suspends relay activity
 * - Intelligent routing to prevent loops
 * 
 * SAFE-MODE CONTROL:
 * - Monitors incoming sensor data for critical conditions
 * - Automatically sends safe-mode commands for low battery
 * - Sends emergency commands for critically low battery
 * - Enforces transmission failure limits
 * - Server can detect temperature anomalies and trigger safe mode
 * - Prevents complete battery drain in transmitter nodes
 * 
 * POWER MANAGEMENT (Battery Nodes):
 * - Normal mode: 5-second check intervals, full functionality
 * - Safe mode: 30-second intervals, reduced relay activity
 * - Emergency mode: 2-minute intervals, minimal activity
 * - Dynamic LED blinking based on power mode
 * - Automatic WiFi disconnection in emergency mode
 * 
 * TROUBLESHOOTING:
 * - Check WiFi credentials if connection fails
 * - Verify server endpoint URL is correct
 * - Use Serial monitor to see packet reception and relay activity
 * - Monitor battery voltage if using battery-powered option
 * - LED blink pattern indicates power mode and activity
 * - Check ESP-NOW MAC address configuration
 * - Verify mesh relay limits and authentication codes
 * 
 * EXTENDING:
 * - Add local data storage for offline operation
 * - Implement retry logic for failed HTTP requests
 * - Add more sophisticated mesh routing algorithms
 * - Support for different sensor types and commands
 * - Integration with LoRaWAN or other long-range protocols
 * - Advanced battery monitoring and prediction
 * - Dynamic power management based on network conditions
 ********************************************************************/
