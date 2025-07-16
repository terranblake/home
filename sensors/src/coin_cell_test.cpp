#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <driver/adc.h>
#include <esp_sleep.h>
#include <esp_wifi.h>
#include <esp_bt.h>
#include "esp_log.h"
#include <HTTPClient.h> // For gateway HTTP functionality
#include <ArduinoJson.h> // For JSON formatting

/*
 * ESP32-C3 Coin Cell Powered Temperature Sensor
 * ============================================
 * 
 * Circuit Diagram:
 * ---------------
 * 
 * CR2450 Battery (3.0V, ~620mAh)
 *    (+) ----+--- 100μF Ceramic Cap ---+--- ESP32-C3 3V3
 *            |                         |
 *            +--- 47μF Electrolytic ---+
 *    (-) ------------------------------- ESP32-C3 GND
 * 
 * NTC Thermistor Circuit (NTCLE100E3103JB0):
 *    ESP32-C3 3V3 --- 10kΩ Resistor ---+--- GPIO0 (ADC1_CH0)
 *                                      |
 *                                      +--- 10kΩ NTC Thermistor --- ESP32-C3 GND
 * 
 * Battery Monitoring Circuit:
 *    ESP32-C3 3V3 --- 100kΩ Resistor ---+--- GPIO1 (ADC1_CH1)
 *                                       |
 *                                       +--- 100kΩ Resistor --- ESP32-C3 GND
 * 
 * Power Saving Features:
 * - CPU frequency reduced to 10 MHz when not transmitting
 * - WiFi in power-save mode when not actively transmitting
 * - Bluetooth disabled
 * - Configurable transmission intervals
 * 
 * Battery Life Estimates (CR2450 @ 620mAh):
 * - Active mode current: ~50mA (during ESP-NOW transmission)
 * - Low power mode: ~5mA (CPU at 10MHz)
 * - 1 minute interval: ~14 days
 * - 5 minute interval: ~64 days 
 * - 15 minute interval: ~150 days
 * - 60 minute interval: ~240 days
 * 
 * ESP-NOW Configuration Commands:
 * - Sleep interval configuration
 * - CPU frequency configuration
 * - Thermistor calibration
 * - Battery monitoring calibration
 * - MAC address configuration
 * - Device enable/disable
 * - Active Mode:
 * - CPU at 10 MHz: ~15 mA
 * - WiFi/ESP-NOW active: ~100 mA
 * - ADC reading: ~1 mA
 * - Active time per cycle: ~1-2 seconds
 * 
 * Expected Battery Life:
 * - 1-minute interval: ~5-6 days
 * - 5-minute interval: ~30 days (1 month)
 * - 15-minute interval: ~90 days (3 months)
 * - 60-minute interval: ~340 days (11 months)
 */

// ESP logging tag - minimal logging in production
static const char *TAG = "COIN_CELL_TEST";

// Operating mode - automatically detected based on power source
typedef enum {
  MODE_SENSOR,           // Low-power sensor mode (coin cell)
  MODE_RECEIVER_GATEWAY  // High-performance receiver/gateway mode (USB/wall)
} device_mode_t;

// Current operating mode - detected at startup
RTC_DATA_ATTR device_mode_t currentMode = MODE_SENSOR;

// Server configuration for gateway mode
#define SERVER_URL "http://192.168.8.123:8080/ingest"  // Server endpoint
#define WIFI_SSID "FromTheLandOfKansas"                // WiFi SSID for gateway mode
#define WIFI_PASSWORD "Kansas_6614!"                   // WiFi password for gateway mode
bool serverConnected = false;                          // WiFi connection status

// Configuration - These can be updated via commands
// Hardware configuration
#define SENSOR_PIN       0      // GPIO0 - ADC1_CH0
#define BATTERY_PIN      1      // GPIO1 - ADC1_CH1

// Sleep configuration
#define DEFAULT_SLEEP_INTERVAL 5  // Default sleep interval in minutes
// Deep sleep time in microseconds (default: 5 minutes)
RTC_DATA_ATTR uint32_t SLEEP_TIME_US = 300000000;  // 5 * 60 * 1000000

// CPU frequency configuration
RTC_DATA_ATTR uint8_t CPU_FREQ_LOW = 10;      // MHz, lowest stable value
RTC_DATA_ATTR uint8_t CPU_FREQ_NORMAL = 80;   // MHz, standard for WiFi

// NTCLE100E3103JB0 thermistor constants
RTC_DATA_ATTR float THERMISTOR_B_VALUE = 3977.0;  // B value from datasheet
RTC_DATA_ATTR float THERMISTOR_R0 = 10000.0;      // Resistance at 25°C (T0)
RTC_DATA_ATTR float THERMISTOR_T0 = 298.15;       // 25°C in Kelvin
RTC_DATA_ATTR float PULLUP_R = 10000.0;           // Pull-up resistor value in ohms

// Battery monitoring constants
RTC_DATA_ATTR float BATT_DIVIDER_RATIO = 2.0;     // Voltage divider ratio

// Device mode configuration
RTC_DATA_ATTR bool DEVICE_ENABLED = true;         // Device on/off state

// Power management functions
void disableUnusedPeripherals() {
  // Disable Bluetooth
  ESP_LOGI(TAG, "Disabling Bluetooth");
  esp_err_t bt_result = esp_bt_controller_disable();
  if (bt_result != ESP_OK) {
    ESP_LOGI(TAG, "Error disabling Bluetooth: %d", bt_result);
  }
  
  // In sensor mode, set WiFi to minimum power
  // In receiver mode, WiFi remains in normal mode for communication
  if (currentMode == MODE_SENSOR) {
    ESP_LOGI(TAG, "Setting WiFi to minimum power mode");
    esp_err_t wifi_result = esp_wifi_set_ps(WIFI_PS_MAX_MODEM);
    if (wifi_result != ESP_OK) {
      ESP_LOGI(TAG, "Error setting WiFi power mode: %d", wifi_result);
    }
  } else {
    ESP_LOGI(TAG, "WiFi in normal power mode for receiver operation");
  }
}

// Simple version of getBatteryVoltage for early device detection
inline uint16_t _getBatteryVoltage(uint16_t raw_adc) {
  // Basic calculation for early detection, uses fixed divider ratio of 2.0
  return raw_adc * (3300.0 / 4095.0) * 2.0;
}

// Detect power source and determine operating mode
device_mode_t detectPowerSource() {
  // Read battery voltage
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_1, ADC_ATTEN_DB_12);
  uint16_t raw_reading = adc1_get_raw(ADC1_CHANNEL_1);
  uint16_t voltage = _getBatteryVoltage(raw_reading);
  
  // USB power is typically 5V, will read higher than coin cell
  // A fully charged battery pack could still read high, so we use a conservative threshold
  if (voltage > 4500) { // Above 4.5V indicates USB power
    ESP_LOGI(TAG, "USB/Wall power detected (Voltage: %d mV) - Switching to RECEIVER mode", voltage);
    return MODE_RECEIVER_GATEWAY;
  } else {
    ESP_LOGI(TAG, "Battery power detected (Voltage: %d mV) - Using SENSOR mode", voltage);
    return MODE_SENSOR;
  }
}

// Connect to WiFi for gateway functionality
bool connectToWiFi() {
  if (currentMode != MODE_RECEIVER_GATEWAY) {
    ESP_LOGI(TAG, "WiFi connection only available in receiver mode");
    return false;
  }
  
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  ESP_LOGI(TAG, "Connecting to WiFi: %s", WIFI_SSID);
  
  // Wait for connection with timeout
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    ESP_LOGI(TAG, ".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    ESP_LOGI(TAG, "WiFi connected");
    ESP_LOGI(TAG, "IP address: %s", WiFi.localIP().toString().c_str());
    return true;
  } else {
    ESP_LOGI(TAG, "WiFi connection failed");
    // Return to ESP-NOW only mode
    WiFi.disconnect();
    WiFi.mode(WIFI_STA);
    return false;
  }
}

void setCpuFrequencyLow() {
  // In sensor mode, set CPU to lowest possible (10MHz)
  // In receiver mode, keep CPU at higher frequency for faster processing
  if (currentMode == MODE_SENSOR) {
    setCpuFrequencyMhz(CPU_FREQ_LOW);
    ESP_LOGI(TAG, "Setting CPU to low power mode: %d MHz", CPU_FREQ_LOW);
  } else {
    // For receiver, we maintain at least 80MHz for WiFi and processing
    setCpuFrequencyMhz(80);
    ESP_LOGI(TAG, "Receiver mode - maintaining CPU at 80 MHz");
  }
}

void setCpuFrequencyNormal() {
  // Set CPU frequency back to normal (80MHz)
  if (currentMode == MODE_SENSOR) {
    setCpuFrequencyMhz(CPU_FREQ_NORMAL);
    ESP_LOGI(TAG, "Setting CPU to normal mode: %d MHz", CPU_FREQ_NORMAL);
  } else {
    // For receiver, we can go higher for better performance
    setCpuFrequencyMhz(160); // ESP32-C3 supports up to 160MHz
    ESP_LOGI(TAG, "Receiver mode - setting CPU to high performance: 160 MHz");
  }
}

// Variables for storing readings
static uint16_t adc_reading;
static uint16_t battery_reading;

// ESP-NOW Receiver MAC Address - update with your receiver's MAC
// Use a string format for easier configuration
// Try both MACs to see if either works
const char* usbPoweredMacStr = "94:A9:90:97:23:08"; // USB-powered device
const char* coinPoweredMacStr = "64:E8:33:B5:F9:E0"; // Coin-powered device
// We'll use broadcast MAC first to see if anything responds
const char* receiverMacStr = "FF:FF:FF:FF:FF:FF"; // Broadcast MAC
uint8_t receiverMac[6];

// ESP-NOW data structure
typedef struct {
  uint32_t node_id;               // Node identifier
  uint16_t raw_temp_adc;          // Raw ADC value from thermistor
  uint16_t battery_mv;            // Battery voltage in millivolts
  uint32_t counter;               // Simple counter
} sensor_data_t;

// Forward declarations
uint16_t getBatteryVoltage(uint16_t raw_adc);
float calculateTemperature(uint16_t raw_adc);
void readSensors();
bool forwardToServer(const sensor_data_t *sensorData, const uint8_t *senderMac);

// Function to convert MAC address string to byte array
void macStringToBytes(const char* macStr, uint8_t* macBytes) {
  // Parse the MAC address string (format: "XX:XX:XX:XX:XX:XX")
  uint8_t index = 0;
  uint8_t temp = 0;
  char c;
  
  for (uint8_t i = 0; i < strlen(macStr); i++) {
    c = macStr[i];
    
    if (c == ':') {
      continue; // Skip separators
    }
    
    temp *= 16; // Shift left by 4 bits
    
    if (c >= '0' && c <= '9') {
      temp += c - '0';
    } else if (c >= 'A' && c <= 'F') {
      temp += c - 'A' + 10;
    } else if (c >= 'a' && c <= 'f') {
      temp += c - 'a' + 10;
    }
    
    if (i % 3 == 1) { // After second character of each byte
      macBytes[index++] = temp;
      temp = 0;
    }
  }
}

// ESP-NOW command types for configuration

// ESP-NOW command types for configuration
typedef enum {
  CMD_NONE = 0,                  // No command
  CMD_SET_SLEEP_INTERVAL = 1,    // Set sleep interval (minutes)
  CMD_SET_CPU_FREQ = 2,          // Set CPU frequency (MHz)
  CMD_SET_THERMISTOR_CAL = 3,    // Set thermistor calibration (B-value, R0)
  CMD_SET_BATTERY_CAL = 4,       // Set battery calibration (divider ratio)
  CMD_SET_DEVICE_ENABLE = 5,     // Enable/disable device
  CMD_SET_MAC_ADDRESS = 6,       // Set receiver MAC address
  CMD_GET_CONFIG = 7,            // Request current configuration
  CMD_GET_STATUS = 8,            // Request current status
  CMD_RESET = 9,                 // Reset device
} command_type_t;

// ESP-NOW command structure
typedef struct {
  uint32_t node_id;              // Target node ID (0xFFFFFFFF for broadcast)
  command_type_t cmd_type;       // Command type
  union {
    struct {                     // For sleep interval
      uint32_t sleep_minutes;    // Sleep interval in minutes
    } sleep;
    struct {                     // For CPU frequency
      uint8_t cpu_freq_low;      // Low CPU frequency in MHz
      uint8_t cpu_freq_normal;   // Normal CPU frequency in MHz
    } cpu;
    struct {                     // For thermistor calibration
      float b_value;             // B-value
      float r0;                  // Resistance at 25°C
      float pullup_r;            // Pullup resistor value
    } therm;
    struct {                     // For battery calibration
      float divider_ratio;       // Voltage divider ratio
    } batt;
    struct {                     // For device enable/disable
      bool enabled;              // Enable state
    } device;
    struct {                     // For MAC address
      uint8_t mac[6];            // New receiver MAC address
    } mac;
  } data;
} command_t;

// Get battery voltage in mV from ADC reading
uint16_t getBatteryVoltage(uint16_t raw_adc) {
  // For a voltage divider with 2x 100K resistors (1:1 ratio)
  // Full range is 0-3.3V, converted to 0-4095
  // Formula: voltage = (raw_adc * 3300 / 4095) * BATT_DIVIDER_RATIO
  return raw_adc * (3300.0 / 4095.0) * BATT_DIVIDER_RATIO;
}

// Calculate temperature in Celsius from thermistor ADC reading
float calculateTemperature(uint16_t raw_adc) {
  // Formula: R_thermistor = PULLUP_R * (4095 / raw_adc - 1)
  float r_thermistor;
  
  // Prevent division by zero
  if (raw_adc == 0) {
    ESP_LOGI(TAG, "Warning: Zero ADC reading, assuming max resistance");
    r_thermistor = 1000000; // Arbitrary high value
  } else {
    r_thermistor = PULLUP_R * (4095.0 / raw_adc - 1.0);
  }
  
  // Calculate temperature using B-parameter equation:
  // 1/T = 1/T0 + (1/B) * ln(R/R0)
  float steinhart = log(r_thermistor / THERMISTOR_R0);
  steinhart /= THERMISTOR_B_VALUE;
  steinhart += 1.0 / THERMISTOR_T0;
  float temp_kelvin = 1.0 / steinhart;
  
  // Convert from Kelvin to Celsius
  float temp_celsius = temp_kelvin - 273.15;
  
  // Sanity check
  if (temp_celsius < -50.0 || temp_celsius > 150.0) {
    ESP_LOGI(TAG, "Warning: Temperature calculation out of reasonable range: %.1f°C", temp_celsius);
  }
  
  return temp_celsius;
}

// Global variable to track ESP-NOW transmission success
static bool lastTransmitSuccess = false;

// Handle ESP-NOW send callback
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Update the status flag
  lastTransmitSuccess = (status == ESP_NOW_SEND_SUCCESS);
  ESP_LOGI(TAG, "ESP-NOW send status: %s", lastTransmitSuccess ? "Success" : "Failed");
}

// Handle ESP-NOW receive callback
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  ESP_LOGI(TAG, "Data received from MAC: %s", macStr);
  
  // If data matches sensor data struct
  if (data_len == sizeof(sensor_data_t)) {
    sensor_data_t *receivedData = (sensor_data_t *)data;
    float temp_celsius = calculateTemperature(receivedData->raw_temp_adc);
    
    ESP_LOGI(TAG, "Received sensor data - Node: 0x%08X, Temp: %.1f°C, Batt: %d mV, Counter: %d", 
             receivedData->node_id, temp_celsius, 
             receivedData->battery_mv, receivedData->counter);
    
    // If we're in receiver/gateway mode, forward to server
    if (currentMode == MODE_RECEIVER_GATEWAY) {
      ESP_LOGI(TAG, "Gateway mode - forwarding data to server");
      bool success = forwardToServer(receivedData, mac_addr);
      if (success) {
        ESP_LOGI(TAG, "Successfully forwarded data to server");
      } else {
        ESP_LOGI(TAG, "Failed to forward data to server");
      }
    }
  } 
  // If data matches command struct
  else if (data_len == sizeof(command_t)) {
    command_t *receivedCommand = (command_t *)data;
    ESP_LOGI(TAG, "Received command - Node ID: 0x%08X, Command: %d", 
             receivedCommand->node_id, receivedCommand->cmd_type);
    
    // Handle commands based on type
    switch (receivedCommand->cmd_type) {
      case CMD_SET_SLEEP_INTERVAL:
        ESP_LOGI(TAG, "Set sleep interval to %d minutes", 
                 receivedCommand->data.sleep.sleep_minutes);
        SLEEP_TIME_US = receivedCommand->data.sleep.sleep_minutes * 60 * 1000000;
        break;
      case CMD_SET_CPU_FREQ:
        ESP_LOGI(TAG, "Set CPU frequency to %d MHz (low), %d MHz (normal)", 
                 receivedCommand->data.cpu.cpu_freq_low, 
                 receivedCommand->data.cpu.cpu_freq_normal);
        CPU_FREQ_LOW = receivedCommand->data.cpu.cpu_freq_low;
        CPU_FREQ_NORMAL = receivedCommand->data.cpu.cpu_freq_normal;
        break;
      case CMD_SET_THERMISTOR_CAL:
        ESP_LOGI(TAG, "Set thermistor calibration - B-value: %.2f, R0: %.2f", 
                 receivedCommand->data.therm.b_value, 
                 receivedCommand->data.therm.r0);
        THERMISTOR_B_VALUE = receivedCommand->data.therm.b_value;
        THERMISTOR_R0 = receivedCommand->data.therm.r0;
        break;
      case CMD_SET_BATTERY_CAL:
        ESP_LOGI(TAG, "Set battery calibration - Divider ratio: %.2f", 
                 receivedCommand->data.batt.divider_ratio);
        BATT_DIVIDER_RATIO = receivedCommand->data.batt.divider_ratio;
        break;
      case CMD_SET_DEVICE_ENABLE:
        ESP_LOGI(TAG, "Set device enable state to %s", 
                 receivedCommand->data.device.enabled ? "Enabled" : "Disabled");
        DEVICE_ENABLED = receivedCommand->data.device.enabled;
        break;
      case CMD_SET_MAC_ADDRESS:
      {
        ESP_LOGI(TAG, "Setting receiver MAC to %02X:%02X:%02X:%02X:%02X:%02X",
                 receivedCommand->data.mac.mac[0], receivedCommand->data.mac.mac[1], 
                 receivedCommand->data.mac.mac[2], receivedCommand->data.mac.mac[3], 
                 receivedCommand->data.mac.mac[4], receivedCommand->data.mac.mac[5]);
        
        // Remove old peer if it exists
        if (esp_now_is_peer_exist(receiverMac)) {
          esp_now_del_peer(receiverMac);
        }
        
        // Update MAC address
        memcpy(receiverMac, receivedCommand->data.mac.mac, 6);
        
        // Add new peer
        esp_now_peer_info_t peerInfo = {};
        memcpy(peerInfo.peer_addr, receiverMac, 6);
        peerInfo.channel = 0;
        peerInfo.encrypt = false;
        peerInfo.ifidx = WIFI_IF_STA;
        
        esp_err_t result = esp_now_add_peer(&peerInfo);
        if (result == ESP_OK) {
          ESP_LOGI(TAG, "New receiver peer added successfully");
        } else {
          ESP_LOGI(TAG, "Failed to add new receiver peer: %d", result);
        }
        break;
      }
      case CMD_GET_CONFIG:
      {
        ESP_LOGI(TAG, "Get configuration command received, sending back configuration");
        
        // Create and send a sensor_data_t packet with our configuration data
        // This reuses the existing data structure but with special values
        sensor_data_t configData;
        configData.node_id = 0x12345678;  // Our node ID
        configData.raw_temp_adc = 0xC0FF;  // Special marker for config data
        configData.battery_mv = 0xEE;      // Special marker
        
        // Embed sleep interval in counter field
        configData.counter = SLEEP_TIME_US / (60 * 1000000); // Convert to minutes
        
        // Send directly back to the requester
        esp_err_t result = esp_now_send(mac_addr, (uint8_t*)&configData, sizeof(configData));
        if (result == ESP_OK) {
          ESP_LOGI(TAG, "Configuration sent back successfully");
        } else {
          ESP_LOGI(TAG, "Failed to send configuration: %d", result);
        }
        
        // If in gateway mode, also forward the configuration to server
        if (currentMode == MODE_RECEIVER_GATEWAY && serverConnected) {
          // Create a JSON document with full configuration
          JsonDocument doc;
          doc["type"] = "config";
          doc["node_id"] = 0x12345678;
          doc["sleep_minutes"] = SLEEP_TIME_US / (60 * 1000000);
          doc["cpu_freq_low"] = CPU_FREQ_LOW;
          doc["cpu_freq_normal"] = CPU_FREQ_NORMAL;
          doc["thermistor_b"] = THERMISTOR_B_VALUE;
          doc["thermistor_r0"] = THERMISTOR_R0;
          doc["thermistor_pullup"] = PULLUP_R;
          doc["battery_divider"] = BATT_DIVIDER_RATIO;
          doc["device_enabled"] = DEVICE_ENABLED;
          doc["operating_mode"] = (currentMode == MODE_SENSOR) ? "sensor" : "gateway";
          
          // Send to server
          HTTPClient http;
          http.begin(SERVER_URL);
          http.addHeader("Content-Type", "application/json");
          
          String jsonString;
          serializeJson(doc, jsonString);
          
          int httpResponseCode = http.POST(jsonString);
          if (httpResponseCode > 0) {
            ESP_LOGI(TAG, "Config sent to server, code: %d", httpResponseCode);
          } else {
            ESP_LOGI(TAG, "Error sending config to server: %d", httpResponseCode);
          }
          
          http.end();
        }
        break;
      }
        
      case CMD_GET_STATUS:
      {
        ESP_LOGI(TAG, "Get status command received, sending back current status");
        
        // Take fresh readings
        readSensors();
        float temp_celsius = calculateTemperature(adc_reading);
        
        // Send our current status back to the sender
        sensor_data_t statusData;
        statusData.node_id = 0x12345678;  // Our node ID
        statusData.raw_temp_adc = adc_reading;
        statusData.battery_mv = getBatteryVoltage(battery_reading);
        statusData.counter = millis();  // Use counter field for uptime in ms
        
        // Send the status data back to the sender
        esp_err_t result = esp_now_send(mac_addr, (uint8_t *)&statusData, sizeof(statusData));
        if (result == ESP_OK) {
          ESP_LOGI(TAG, "Status sent back successfully");
        } else {
          ESP_LOGI(TAG, "Failed to send status: %d", result);
        }
        
        // If in gateway mode, also send detailed status to server
        if (currentMode == MODE_RECEIVER_GATEWAY && serverConnected) {
          // Create a JSON document with full status
          JsonDocument doc;
          doc["type"] = "status";
          doc["node_id"] = 0x12345678;
          doc["mac_address"] = WiFi.macAddress();
          doc["temperature_c"] = temp_celsius;
          doc["raw_adc"] = adc_reading;
          doc["battery_mv"] = getBatteryVoltage(battery_reading);
          doc["uptime_ms"] = millis();
          doc["operating_mode"] = "gateway";
          doc["cpu_freq"] = getCpuFrequencyMhz();
          doc["wifi_rssi"] = WiFi.RSSI();
          doc["wifi_ip"] = WiFi.localIP().toString();
          
          // Send to server
          HTTPClient http;
          http.begin(SERVER_URL);
          http.addHeader("Content-Type", "application/json");
          
          String jsonString;
          serializeJson(doc, jsonString);
          
          int httpResponseCode = http.POST(jsonString);
          if (httpResponseCode > 0) {
            ESP_LOGI(TAG, "Status sent to server, code: %d", httpResponseCode);
          } else {
            ESP_LOGI(TAG, "Error sending status to server: %d", httpResponseCode);
          }
          
          http.end();
        }
        break;
      }
      case CMD_RESET:
      {
        ESP_LOGI(TAG, "Reset command received, restarting device in 2 seconds...");
        delay(2000);
        ESP.restart();
        break;
      }
      default:
        ESP_LOGI(TAG, "Unknown command type: %d", receivedCommand->cmd_type);
        break;
    }
  }
}

// Relay a command to a specific node
bool relayCommandToNode(uint32_t targetNodeId, command_type_t cmdType, void* cmdData) {
  if (currentMode != MODE_RECEIVER_GATEWAY) {
    ESP_LOGI(TAG, "Command relay only available in gateway mode");
    return false;
  }
  
  // Create command
  command_t command;
  command.node_id = targetNodeId;
  command.cmd_type = cmdType;
  
  // Fill in command data based on type
  switch (cmdType) {
    case CMD_SET_SLEEP_INTERVAL:
      command.data.sleep.sleep_minutes = *((uint32_t*)cmdData);
      break;
      
    case CMD_SET_CPU_FREQ:
      command.data.cpu.cpu_freq_low = ((uint8_t*)cmdData)[0];
      command.data.cpu.cpu_freq_normal = ((uint8_t*)cmdData)[1];
      break;
      
    case CMD_SET_THERMISTOR_CAL:
      command.data.therm.b_value = ((float*)cmdData)[0];
      command.data.therm.r0 = ((float*)cmdData)[1];
      command.data.therm.pullup_r = ((float*)cmdData)[2];
      break;
      
    case CMD_SET_BATTERY_CAL:
      command.data.batt.divider_ratio = *((float*)cmdData);
      break;
      
    case CMD_SET_DEVICE_ENABLE:
      command.data.device.enabled = *((bool*)cmdData);
      break;
      
    case CMD_SET_MAC_ADDRESS:
      memcpy(command.data.mac.mac, cmdData, 6);
      break;
      
    case CMD_GET_CONFIG:
    case CMD_GET_STATUS:
    case CMD_RESET:
      // These commands don't need additional data
      break;
      
    default:
      ESP_LOGI(TAG, "Unknown command type for relay");
      return false;
  }
  
  // Send the command to broadcast or to specific MAC if we have it
  // For now, we'll broadcast
  uint8_t broadcastMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  
  // Register broadcast peer if not already
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA;
  
  if (esp_now_is_peer_exist(broadcastMac)) {
    esp_now_del_peer(broadcastMac);
  }
  esp_now_add_peer(&peerInfo);
  
  // Send command
  esp_err_t result = esp_now_send(broadcastMac, (uint8_t*)&command, sizeof(command));
  
  if (result == ESP_OK) {
    ESP_LOGI(TAG, "Relayed command type %d to node 0x%08X", cmdType, targetNodeId);
    return true;
  } else {
    ESP_LOGI(TAG, "Failed to relay command: %d", result);
    return false;
  }
}

// Setup ESP-NOW
void setupEspNow() {
  // Initialize WiFi in Station mode
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();  // Disconnect from any AP
  delay(100);         // Short delay to ensure disconnection completes
  
  ESP_LOGI(TAG, "Initializing ESP-NOW...");
  if (esp_now_init() != ESP_OK) {
    ESP_LOGI(TAG, "Error initializing ESP-NOW");
    return;
  }
  ESP_LOGI(TAG, "ESP-NOW initialized successfully");
  
  // Register callbacks
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  
  // Register peer with more robust error handling
  esp_now_peer_info_t peerInfo = {};  // Initialize all fields to zero
  memcpy(peerInfo.peer_addr, receiverMac, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA;  // Set interface explicitly
  
  // Check if peer exists before adding
  bool exists = esp_now_is_peer_exist(receiverMac);
  if (exists) {
    ESP_LOGI(TAG, "Peer already exists, removing first");
    esp_now_del_peer(receiverMac);
  }
  
  esp_err_t result = esp_now_add_peer(&peerInfo);
  if (result != ESP_OK) {
    ESP_LOGI(TAG, "Failed to add peer, error: %d", result);
    return;
  }
  ESP_LOGI(TAG, "Peer added successfully");
}

// Send data using ESP-NOW
void sendEspNowData() {
  static uint32_t counter = 0;
  static const uint32_t NODE_ID = 0x12345678;  // Device unique identifier
  sensor_data_t sensorData;
  
  // Fill data structure with actual readings
  sensorData.node_id = NODE_ID;
  sensorData.raw_temp_adc = adc_reading;
  sensorData.battery_mv = getBatteryVoltage(battery_reading);
  sensorData.counter = counter++;  // Increment counter
  
  // Calculate temperature for logging
  float temp_celsius = calculateTemperature(adc_reading);
  
  // Send data
  esp_err_t result = esp_now_send(receiverMac, (uint8_t *)&sensorData, sizeof(sensorData));
  
  if (result == ESP_OK) {
    ESP_LOGI(TAG, "ESP-NOW packet sent successfully (API call OK)");
    ESP_LOGI(TAG, "Packet: node_id=0x%X, adc=%d (%.1f°C), battery=%d mV, counter=%d", 
             sensorData.node_id, sensorData.raw_temp_adc, temp_celsius,
             sensorData.battery_mv, sensorData.counter);
  } else {
    ESP_LOGI(TAG, "Error sending the ESP-NOW packet: %d", result);
  }
}

// Configure and read ADC
void setupADC() {
  ESP_LOGI(TAG, "Setting ADC width to 12 bits");
  esp_err_t width_result = adc1_config_width(ADC_WIDTH_BIT_12);
  if (width_result != ESP_OK) {
    ESP_LOGI(TAG, "Error setting ADC width: %d", width_result);
  }
  
  ESP_LOGI(TAG, "Configuring ADC1_CHANNEL_0 (GPIO %d)", SENSOR_PIN);
  esp_err_t ch0_result = adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_12); // For thermistor
  if (ch0_result != ESP_OK) {
    ESP_LOGI(TAG, "Error configuring ADC1_CHANNEL_0: %d", ch0_result);
  }
  
  ESP_LOGI(TAG, "Configuring ADC1_CHANNEL_1 (GPIO %d)", BATTERY_PIN);
  esp_err_t ch1_result = adc1_config_channel_atten(ADC1_CHANNEL_1, ADC_ATTEN_DB_12); // For battery
  if (ch1_result != ESP_OK) {
    ESP_LOGI(TAG, "Error configuring ADC1_CHANNEL_1: %d", ch1_result);
  }
  
  ESP_LOGI(TAG, "ADC configuration complete");
}

// Read the ADC values
void readSensors() {
  // Acquire ADC power - only power when needed
  adc_power_acquire();
  
  // Read thermistor (averaged for stability)
  uint32_t temp_sum = 0;
  for (int i = 0; i < 5; i++) {
    temp_sum += adc1_get_raw(ADC1_CHANNEL_0);
    delay(10);  // Short delay between readings
  }
  adc_reading = temp_sum / 5;
  
  // Read battery (averaged for stability)
  uint32_t batt_sum = 0;
  for (int i = 0; i < 5; i++) {
    batt_sum += adc1_get_raw(ADC1_CHANNEL_1);
    delay(10);  // Short delay between readings
  }
  battery_reading = batt_sum / 5;
  
  // Release ADC power when done
  adc_power_release();
  
  // Check for valid readings
  if (adc_reading < 100 || battery_reading < 100) {
    ESP_LOGI(TAG, "Warning: Low ADC readings - Thermistor: %d, Battery: %d", 
             adc_reading, battery_reading);
  } else {
    uint16_t batt_mv = getBatteryVoltage(battery_reading);
    float temp_celsius = calculateTemperature(adc_reading);
    
    ESP_LOGI(TAG, "Readings - Temp: %.1f°C, Battery: %d mV", 
             temp_celsius, batt_mv);
  }
}

// Delay between transmissions
void delaySend() {
  uint32_t delay_ms = 5000;  // Default 5 seconds
  
  // If device is enabled, use the configured sleep time (convert from microseconds to milliseconds)
  if (DEVICE_ENABLED) {
    // Check for reasonable value (between 1 second and 10 minutes)
    uint32_t sleep_ms = SLEEP_TIME_US / 1000;
    if (sleep_ms > 1000 && sleep_ms < 600000) {
      delay_ms = sleep_ms;
    }
  }
  
  ESP_LOGI(TAG, "Waiting %d ms before next transmission...", delay_ms);
  delay(delay_ms);
}

// Forward sensor data to server in gateway mode
bool forwardToServer(const sensor_data_t *sensorData, const uint8_t *senderMac) {
  if (currentMode != MODE_RECEIVER_GATEWAY || !serverConnected) {
    return false; // Only attempt in gateway mode with connection
  }
  
  // Check WiFi connection
  if (WiFi.status() != WL_CONNECTED) {
    ESP_LOGI(TAG, "WiFi disconnected, attempting to reconnect");
    serverConnected = connectToWiFi();
    if (!serverConnected) {
      return false;
    }
  }
  
  // Create JSON document
  JsonDocument doc;
  
  // Format MAC address as string
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           senderMac[0], senderMac[1], senderMac[2], senderMac[3], senderMac[4], senderMac[5]);
  
  // Add sensor data to JSON
  doc["node_id"] = sensorData->node_id;
  doc["mac_address"] = macStr;
  doc["raw_temp"] = sensorData->raw_temp_adc;
  
  // Calculate temperature
  float temperature = calculateTemperature(sensorData->raw_temp_adc);
  doc["temperature_c"] = temperature;
  
  doc["battery_mv"] = sensorData->battery_mv;
  doc["counter"] = sensorData->counter;
  doc["gateway"] = WiFi.macAddress();
  doc["timestamp"] = millis();
  
  // Serialize JSON to string
  String jsonString;
  serializeJson(doc, jsonString);
  
  // Send HTTP POST request
  HTTPClient http;
  http.begin(SERVER_URL);
  http.addHeader("Content-Type", "application/json");
  
  int httpResponseCode = http.POST(jsonString);
  
  if (httpResponseCode > 0) {
    ESP_LOGI(TAG, "HTTP Response code: %d", httpResponseCode);
    String payload = http.getString();
    ESP_LOGI(TAG, "Server response: %s", payload.c_str());
    http.end();
    return true;
  } else {
    ESP_LOGI(TAG, "HTTP Error: %d", httpResponseCode);
    http.end();
    return false;
  }
}

void setup() {
  // Initialize serial for debugging
  Serial.begin(115200);
  delay(1000);
  
  ESP_LOGI(TAG, "************ COIN CELL TEST - ULTRA LOW POWER MODE ************");
  
  // Print device MAC address
  uint8_t mac[6];
  esp_efuse_mac_get_default(mac);
  ESP_LOGI(TAG, "Device MAC: %02X:%02X:%02X:%02X:%02X:%02X", 
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  
  // Set initial receiver MAC (broadcast)
  macStringToBytes(receiverMacStr, receiverMac);
  
  // Initialize ADC
  setupADC();
  
  // Take initial readings
  adc_reading = adc1_get_raw(ADC1_CHANNEL_0);
  battery_reading = adc1_get_raw(ADC1_CHANNEL_1);
  float temp_celsius = calculateTemperature(adc_reading);
  uint16_t batt_mv = getBatteryVoltage(battery_reading);
  ESP_LOGI(TAG, "Initial readings - Temp: %.1f°C, Battery: %d mV", 
           temp_celsius, batt_mv);
  
  // Set CPU to normal frequency for WiFi/ESP-NOW setup
  setCpuFrequencyNormal();
  
  // Detect power source and set mode
  currentMode = detectPowerSource();
  ESP_LOGI(TAG, "Detected mode: %s", (currentMode == MODE_SENSOR) ? "SENSOR" : "RECEIVER_GATEWAY");
  
  // Set up ESP-NOW
  setupEspNow();
  
  // Connect to WiFi if in receiver mode
  if (currentMode == MODE_RECEIVER_GATEWAY) {
    connectToWiFi();
  }
  
  // Disable unused peripherals to save power
  disableUnusedPeripherals();
  
  // Lower CPU frequency for power saving
  setCpuFrequencyLow();
  ESP_LOGI(TAG, "Current CPU frequency: %d MHz", getCpuFrequencyMhz());
  
  ESP_LOGI(TAG, "Setup complete, transmission interval: %d minutes", 
          (SLEEP_TIME_US / (60 * 1000000)));
}

void loop() {
  // Check if device is enabled
  if (!DEVICE_ENABLED) {
    ESP_LOGI(TAG, "Device is disabled. Waiting for commands...");
    delay(5000);  // Still need to wake periodically to check for commands
    return;
  }
  
  // Different behavior based on operating mode
  if (currentMode == MODE_SENSOR) {
    // SENSOR MODE OPERATION
    // Increase CPU frequency temporarily for operations
    setCpuFrequencyNormal();
    
    // Read sensor values
    readSensors();
    
    // Ensure WiFi is in station mode for ESP-NOW
    if (WiFi.getMode() != WIFI_STA) {
      WiFi.mode(WIFI_STA);
      delay(100);
    }
    
    // Check if ESP-NOW is initialized
    esp_err_t status = esp_now_init();
    if (status != ESP_OK) {
      ESP_LOGI(TAG, "Reinitializing ESP-NOW");
      setupEspNow();
    }
    
    // Try alternating between broadcast and specific MACs
    static int attempt = 0;
    attempt = (attempt + 1) % 3;
    
    // Set the appropriate MAC address
    if (attempt == 0) {
      macStringToBytes("FF:FF:FF:FF:FF:FF", receiverMac); // Broadcast MAC
    } else if (attempt == 1) {
      macStringToBytes(usbPoweredMacStr, receiverMac);    // USB-powered receiver
    } else {
      macStringToBytes(coinPoweredMacStr, receiverMac);   // Coin-powered receiver
    }
    
    // Update peer with the new MAC address
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, receiverMac, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    peerInfo.ifidx = WIFI_IF_STA;
    
    if (esp_now_is_peer_exist(receiverMac)) {
      esp_now_del_peer(receiverMac);
    }
    esp_now_add_peer(&peerInfo);
    
    // Send data
    sendEspNowData();
    
    // Wait for transmission to complete
    delay(100);
    
    // Power saving
    esp_wifi_set_ps(WIFI_PS_MAX_MODEM);  // WiFi power save mode
    setCpuFrequencyLow();  // Lower CPU frequency
    
    // Wait before next transmission
    delaySend();
  } 
  else {
    // RECEIVER/GATEWAY MODE OPERATION
    
    // Keep higher CPU frequency for better performance
    setCpuFrequencyNormal();
    
    // Check WiFi connectivity if we're supposed to be connected
    if (serverConnected && WiFi.status() != WL_CONNECTED) {
      ESP_LOGI(TAG, "Gateway mode - WiFi disconnected, reconnecting...");
      serverConnected = connectToWiFi();
    }
    
    // In receiver mode, we just listen for ESP-NOW messages and forward them
    // We still read our own sensors periodically to monitor the gateway itself
    static unsigned long lastSensorReadTime = 0;
    if (millis() - lastSensorReadTime > 60000) { // Once per minute
      readSensors();
      float temp_celsius = calculateTemperature(adc_reading);
      uint16_t batt_mv = getBatteryVoltage(battery_reading);
      ESP_LOGI(TAG, "Gateway readings - Temp: %.1f°C, Battery: %d mV", 
              temp_celsius, batt_mv);
      lastSensorReadTime = millis();
    }
    
    // Just a brief delay to prevent CPU hogging
    delay(100);
  }
}
