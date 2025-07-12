/********************************************************************
 * ESP32-C3 Battery Transmitter - RECOVERY/DEBUG VERSION
 * 
 * This version stays awake for debugging and testing.
 * Use this to:
 * - Test ESP-NOW communication
 * - Debug sensor readings
 * - Get MAC addresses
 * - Verify circuit operation
 * 
 * ⚠️ WARNING: This will drain your battery quickly!
 * Switch to battery_transmitter.cpp for production use.
 ********************************************************************/

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

/************* USER SETTINGS *****************/
const char* NODE_ID = "debug_01";    // Unique identifier for this sensor
const int SENSOR_PIN = 4;            // GPIO4 for thermistor (ADC1_CH4)
const int NTC_POWER_PIN = 5;         // GPIO5 to control thermistor power (optional)
const int BATTERY_PIN = 3;           // GPIO3 for battery voltage monitoring (ADC1_CH3)

// Test interval in seconds (much shorter for debugging)
const int TEST_INTERVAL = 10;        // Send data every 10 seconds

// Receiver MAC address (use broadcast for initial testing)
uint8_t receiverMAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // Broadcast
/*********************************************/

// Thermistor constants for NTCLE100E3103JB0
constexpr float R_FIXED = 10000.0;   // 10kΩ fixed resistor
constexpr float BETA = 3977.0;       // β25/85 = 3977K
constexpr float T0_TEMP = 298.15;    // 25°C in Kelvin
constexpr float R0 = 10000.0;        // 10kΩ @25°C

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
} sensor_data_t;

// Global variables
uint32_t sequence_number = 0;
sensor_data_t sensor_data;

void printMACAddress() {
  uint8_t baseMac[6];
  esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
  Serial.printf("Transmitter MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\n",
                baseMac[0], baseMac[1], baseMac[2], 
                baseMac[3], baseMac[4], baseMac[5]);
}

float readBatteryVoltage() {
  // Read battery voltage (3V max from CR2450)
  int raw = analogRead(BATTERY_PIN);
  return (raw * 3.0) / 4095.0;  // Convert to voltage
}

float readThermistorC() {
  // Power on the thermistor circuit
  #ifdef NTC_POWER_PIN
  digitalWrite(NTC_POWER_PIN, HIGH);
  delay(10);  // Allow settling time
  Serial.println("NTC powered ON");
  #endif
  
  int raw = analogRead(SENSOR_PIN);
  Serial.printf("Raw ADC: %d\n", raw);
  
  // ESP32-C3 with CR2450 direct connection (no regulator)
  float battery_v = readBatteryVoltage();
  float v = raw * (battery_v / 4095.0);  // Use actual battery voltage as reference
  Serial.printf("Voltage: %.3f V (Battery: %.3f V)\n", v, battery_v);
  
  // Power off the thermistor circuit to save power
  #ifdef NTC_POWER_PIN
  digitalWrite(NTC_POWER_PIN, LOW);
  Serial.println("NTC powered OFF");
  #endif
  
  if (v >= (battery_v - 0.1)) {
    Serial.println("Error: Voltage near supply - check thermistor connection");
    return -999.0;
  }
  
  float rT = (v * R_FIXED) / (battery_v - v);  // Voltage divider math
  Serial.printf("Thermistor resistance: %.1f Ω\n", rT);
  
  float tempK = 1.0 / ((1.0 / T0_TEMP) + (1.0 / BETA) * log(rT / R0));
  return tempK - 273.15;  // Convert to Celsius
}

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.printf("ESP-NOW Send to %02X:%02X:%02X:%02X:%02X:%02X: %s\n",
                mac_addr[0], mac_addr[1], mac_addr[2], 
                mac_addr[3], mac_addr[4], mac_addr[5],
                status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAIL");
}

void initESPNow() {
  // Set WiFi mode and initialize ESP-NOW
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  
  Serial.println("Initializing ESP-NOW...");
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
    return;
  }
  
  // Register send callback
  esp_now_register_send_cb(onDataSent);
  
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
  Serial.printf("Peer added: %02X:%02X:%02X:%02X:%02X:%02X\n",
                receiverMAC[0], receiverMAC[1], receiverMAC[2],
                receiverMAC[3], receiverMAC[4], receiverMAC[5]);
}

void sendSensorData() {
  Serial.println("\n=== Reading Sensors ===");
  
  // Read sensor data
  float temp_c = readThermistorC();
  float temp_f = temp_c * 9.0 / 5.0 + 32.0;
  float battery_v = readBatteryVoltage();
  
  // Prepare data packet
  strncpy(sensor_data.node_id, NODE_ID, sizeof(sensor_data.node_id) - 1);
  sensor_data.temp_c = temp_c;
  sensor_data.temp_f = temp_f;
  sensor_data.battery_v = battery_v;
  sensor_data.sequence = ++sequence_number;
  sensor_data.timestamp = millis();
  sensor_data.raw_adc = analogRead(SENSOR_PIN);
  sensor_data.resistance = 0.0;  // Could calculate if needed
  
  Serial.printf("Node ID: %s\n", sensor_data.node_id);
  Serial.printf("Temperature: %.2f°C (%.2f°F)\n", temp_c, temp_f);
  Serial.printf("Battery: %.3fV\n", battery_v);
  Serial.printf("Sequence: %d\n", sensor_data.sequence);
  Serial.printf("Raw ADC: %d\n", sensor_data.raw_adc);
  
  // Check battery level
  if (battery_v < 2.5) {
    Serial.println("⚠️  LOW BATTERY WARNING!");
  }
  
  Serial.println("\n=== Sending ESP-NOW Packet ===");
  
  // Send data via ESP-NOW
  esp_err_t result = esp_now_send(receiverMAC, 
                                  (uint8_t*)&sensor_data, 
                                  sizeof(sensor_data));
  
  if (result == ESP_OK) {
    Serial.println("ESP-NOW packet queued successfully");
  } else {
    Serial.printf("ESP-NOW send failed: %d\n", result);
  }
  
  // Wait for transmission callback
  delay(200);
  Serial.println("=========================\n");
}

void setup() {
  // Standard CPU frequency for debugging
  setCpuFrequencyMhz(160);
  
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n=== ESP32-C3 Debug Transmitter ===");
  Serial.println("⚠️  RECOVERY MODE - NO DEEP SLEEP ⚠️");
  Serial.printf("Node ID: %s\n", NODE_ID);
  Serial.printf("CPU Frequency: %d MHz\n", getCpuFrequencyMhz());
  Serial.printf("Test interval: %d seconds\n", TEST_INTERVAL);
  
  // Print MAC address
  printMACAddress();
  
  // Configure GPIO pins
  #ifdef NTC_POWER_PIN
  pinMode(NTC_POWER_PIN, OUTPUT);
  digitalWrite(NTC_POWER_PIN, LOW);  // Start with thermistor powered off
  Serial.println("NTC power control enabled on GPIO5");
  #endif
  
  // Test initial readings
  Serial.println("\n=== Initial Sensor Test ===");
  float battery_v = readBatteryVoltage();
  float temp_c = readThermistorC();
  Serial.printf("Battery: %.3fV\n", battery_v);
  Serial.printf("Temperature: %.2f°C\n", temp_c);
  
  if (battery_v < 2.0) {
    Serial.println("❌ BATTERY TOO LOW - CHECK CONNECTION");
  } else if (battery_v < 2.5) {
    Serial.println("⚠️  BATTERY LOW - REPLACE SOON");
  } else {
    Serial.println("✅ BATTERY OK");
  }
  
  if (temp_c < -50 || temp_c > 100) {
    Serial.println("❌ TEMPERATURE READING INVALID - CHECK NTC");
  } else {
    Serial.println("✅ TEMPERATURE READING OK");
  }
  
  // Initialize ESP-NOW
  initESPNow();
  
  Serial.println("\n🎯 Debug mode ready!");
  Serial.println("📡 Sending test packets every " + String(TEST_INTERVAL) + " seconds");
  Serial.println("🔄 Use Serial monitor to observe operation");
  Serial.println("=" * 50);
}

void loop() {
  // Send sensor data
  sendSensorData();
  
  // Wait for next transmission
  Serial.printf("Waiting %d seconds until next transmission...\n\n", TEST_INTERVAL);
  delay(TEST_INTERVAL * 1000);
}

/********************************************************************
 * DEBUG INSTRUCTIONS:
 * 
 * 1. Flash this recovery version to test your setup
 * 2. Open Serial Monitor (115200 baud)
 * 3. Verify sensor readings and battery voltage
 * 4. Check ESP-NOW packet transmission
 * 5. Note the MAC address for receiver configuration
 * 6. Once working, switch to battery_transmitter.cpp
 * 
 * WHAT TO CHECK:
 * ✅ Battery voltage should be 2.5-3.2V
 * ✅ Temperature readings should be reasonable
 * ✅ ESP-NOW packets should send successfully
 * ✅ MAC address is displayed for receiver setup
 * 
 * TROUBLESHOOTING:
 * - Low battery voltage: Check CR2450 connection
 * - Invalid temperature: Check NTC thermistor wiring
 * - ESP-NOW fails: Check receiver is running and MAC address
 * - No response: Check power supply and connections
 * 
 * ⚠️ REMEMBER: This drains battery quickly!
 * Use only for testing, then switch to production version.
 ********************************************************************/
