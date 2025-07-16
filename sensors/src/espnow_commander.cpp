#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <driver/adc.h>
#include <esp_sleep.h>
#include <esp_wifi.h>
#include "esp_log.h"

// ESP-NOW Commander - Test utility to send commands to ESP-NOW nodes
// This sketch allows sending configuration commands to the coin cell test device

// ESP logging tag
static const char *TAG = "ESP_NOW_COMMANDER";

// Structure definitions - must match the coin cell test device
// ESP-NOW data structure
typedef struct {
  uint32_t node_id;               // Node identifier
  uint16_t raw_temp_adc;          // Raw ADC value from thermistor
  uint16_t battery_mv;            // Battery voltage in millivolts
  uint32_t counter;               // Simple counter
} sensor_data_t;

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

// Target MAC addresses - update with your device MACs
const char* targetMacStr = "FF:FF:FF:FF:FF:FF"; // Default to broadcast
uint8_t targetMac[6];  // Will hold the converted MAC address

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

// Global variable to track ESP-NOW transmission success
static bool lastTransmitSuccess = false;

// Handle ESP-NOW send callback
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  lastTransmitSuccess = (status == ESP_NOW_SEND_SUCCESS);
  ESP_LOGI(TAG, "ESP-NOW send status: %s", lastTransmitSuccess ? "Success" : "Failed");
}

// Handle ESP-NOW receive callback
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  ESP_LOGI(TAG, "Data received from MAC: %s", macStr);
  
  // Check if it's a sensor data packet
  if (data_len == sizeof(sensor_data_t)) {
    sensor_data_t *receivedData = (sensor_data_t *)data;
    
    // Check if this is a special config response (marked by special ADC value)
    if (receivedData->raw_temp_adc == 0xC0FF) {
      ESP_LOGI(TAG, "Received CONFIG response from 0x%08X", receivedData->node_id);
      ESP_LOGI(TAG, "Sleep interval: %d minutes", receivedData->counter);
    } 
    // Normal sensor data
    else {
      ESP_LOGI(TAG, "Received sensor data:");
      ESP_LOGI(TAG, "Node ID: 0x%08X", receivedData->node_id);
      ESP_LOGI(TAG, "Raw ADC: %d", receivedData->raw_temp_adc);
      ESP_LOGI(TAG, "Battery: %d mV", receivedData->battery_mv);
      ESP_LOGI(TAG, "Counter: %d", receivedData->counter);
    }
  }
  // Check if it's a command response
  else if (data_len == sizeof(command_t)) {
    command_t *cmd = (command_t *)data;
    ESP_LOGI(TAG, "Received command from Node 0x%08X, Type: %d", 
             cmd->node_id, cmd->cmd_type);
  }
  // Unknown data format
  else {
    ESP_LOGI(TAG, "Received unknown data format (%d bytes)", data_len);
  }
}

// Setup ESP-NOW
void setupEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  
  ESP_LOGI(TAG, "Initializing ESP-NOW");
  if (esp_now_init() != ESP_OK) {
    ESP_LOGI(TAG, "Error initializing ESP-NOW");
    return;
  }
  
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  
  // Register broadcast peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, targetMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA;
  
  if (esp_now_is_peer_exist(targetMac)) {
    esp_now_del_peer(targetMac);
  }
  
  esp_err_t result = esp_now_add_peer(&peerInfo);
  if (result != ESP_OK) {
    ESP_LOGI(TAG, "Failed to add peer: %d", result);
    return;
  }
  
  ESP_LOGI(TAG, "ESP-NOW setup complete");
}

// Send a test command
void sendCommand(command_type_t cmdType) {
  command_t cmd;
  cmd.node_id = 0xFFFFFFFF;  // Broadcast to all nodes
  cmd.cmd_type = cmdType;
  
  // Fill command-specific data
  switch (cmdType) {
    case CMD_SET_SLEEP_INTERVAL:
      cmd.data.sleep.sleep_minutes = 1;  // Set to 1 minute for testing
      ESP_LOGI(TAG, "Sending SET_SLEEP_INTERVAL command: %d minutes", 
               cmd.data.sleep.sleep_minutes);
      break;
      
    case CMD_SET_CPU_FREQ:
      cmd.data.cpu.cpu_freq_low = 10;    // 10 MHz low
      cmd.data.cpu.cpu_freq_normal = 80; // 80 MHz normal
      ESP_LOGI(TAG, "Sending SET_CPU_FREQ command: Low=%d MHz, Normal=%d MHz", 
               cmd.data.cpu.cpu_freq_low, cmd.data.cpu.cpu_freq_normal);
      break;
      
    case CMD_SET_THERMISTOR_CAL:
      cmd.data.therm.b_value = 3977.0;   // Default B value
      cmd.data.therm.r0 = 10000.0;       // Default R0
      cmd.data.therm.pullup_r = 10000.0; // Default pullup
      ESP_LOGI(TAG, "Sending SET_THERMISTOR_CAL command: B=%.1f, R0=%.1f, Pullup=%.1f", 
               cmd.data.therm.b_value, cmd.data.therm.r0, cmd.data.therm.pullup_r);
      break;
      
    case CMD_SET_BATTERY_CAL:
      cmd.data.batt.divider_ratio = 2.0; // Default divider ratio
      ESP_LOGI(TAG, "Sending SET_BATTERY_CAL command: Divider=%.2f", 
               cmd.data.batt.divider_ratio);
      break;
      
    case CMD_SET_DEVICE_ENABLE:
      cmd.data.device.enabled = true;    // Enable device
      ESP_LOGI(TAG, "Sending SET_DEVICE_ENABLE command: %s", 
               cmd.data.device.enabled ? "ENABLE" : "DISABLE");
      break;
      
    case CMD_SET_MAC_ADDRESS:
    {
      // Set to a specific MAC address
      uint8_t newMac[6] = {0x94, 0xA9, 0x90, 0x97, 0x23, 0x08}; // Example MAC
      memcpy(cmd.data.mac.mac, newMac, 6);
      ESP_LOGI(TAG, "Sending SET_MAC_ADDRESS command: %02X:%02X:%02X:%02X:%02X:%02X",
               cmd.data.mac.mac[0], cmd.data.mac.mac[1], cmd.data.mac.mac[2],
               cmd.data.mac.mac[3], cmd.data.mac.mac[4], cmd.data.mac.mac[5]);
      break;
    }
      
    case CMD_GET_CONFIG:
      ESP_LOGI(TAG, "Sending GET_CONFIG command");
      break;
      
    case CMD_GET_STATUS:
      ESP_LOGI(TAG, "Sending GET_STATUS command");
      break;
      
    case CMD_RESET:
      ESP_LOGI(TAG, "Sending RESET command");
      break;
      
    default:
      ESP_LOGI(TAG, "Unknown command type");
      return;
  }
  
  // Send the command
  esp_err_t result = esp_now_send(targetMac, (uint8_t *)&cmd, sizeof(cmd));
  
  if (result == ESP_OK) {
    ESP_LOGI(TAG, "Command sent successfully (API call OK)");
  } else {
    ESP_LOGI(TAG, "Error sending command: %d", result);
  }
  
  // Wait for callback to execute
  delay(100);
}

void printMenu() {
  Serial.println("\n==== ESP-NOW Commander Menu ====");
  Serial.println("1: Set sleep interval (1 minute)");
  Serial.println("2: Set CPU frequencies (10/80 MHz)");
  Serial.println("3: Set thermistor calibration (default values)");
  Serial.println("4: Set battery calibration (2.0 divider ratio)");
  Serial.println("5: Set device enable (TRUE)");
  Serial.println("6: Set device disable (FALSE)");
  Serial.println("7: Set MAC address (94:A9:90:97:23:08)");
  Serial.println("8: Get configuration");
  Serial.println("9: Get status");
  Serial.println("0: Send reset command");
  Serial.println("m: Show this menu");
  Serial.println("================================");
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  ESP_LOGI(TAG, "======== ESP-NOW COMMAND UTILITY ========");
  
  // Get our MAC address
  uint8_t mac[6];
  esp_efuse_mac_get_default(mac);
  ESP_LOGI(TAG, "Commander MAC: %02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  
  // Convert target MAC string to bytes
  macStringToBytes(targetMacStr, targetMac);
  ESP_LOGI(TAG, "Target MAC: %02X:%02X:%02X:%02X:%02X:%02X",
           targetMac[0], targetMac[1], targetMac[2], targetMac[3], targetMac[4], targetMac[5]);
  
  // Set up ESP-NOW
  setupEspNow();
  
  // Show command menu
  printMenu();
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    
    switch (cmd) {
      case '1':
        sendCommand(CMD_SET_SLEEP_INTERVAL);
        break;
      case '2':
        sendCommand(CMD_SET_CPU_FREQ);
        break;
      case '3':
        sendCommand(CMD_SET_THERMISTOR_CAL);
        break;
      case '4':
        sendCommand(CMD_SET_BATTERY_CAL);
        break;
      case '5':
        // Enable device
        {
          command_t cmd;
          cmd.node_id = 0xFFFFFFFF;
          cmd.cmd_type = CMD_SET_DEVICE_ENABLE;
          cmd.data.device.enabled = true;
          ESP_LOGI(TAG, "Sending SET_DEVICE_ENABLE command: ENABLE");
          esp_now_send(targetMac, (uint8_t *)&cmd, sizeof(cmd));
        }
        break;
      case '6':
        // Disable device
        {
          command_t cmd;
          cmd.node_id = 0xFFFFFFFF;
          cmd.cmd_type = CMD_SET_DEVICE_ENABLE;
          cmd.data.device.enabled = false;
          ESP_LOGI(TAG, "Sending SET_DEVICE_ENABLE command: DISABLE");
          esp_now_send(targetMac, (uint8_t *)&cmd, sizeof(cmd));
        }
        break;
      case '7':
        sendCommand(CMD_SET_MAC_ADDRESS);
        break;
      case '8':
        sendCommand(CMD_GET_CONFIG);
        break;
      case '9':
        sendCommand(CMD_GET_STATUS);
        break;
      case '0':
        sendCommand(CMD_RESET);
        break;
      case 'm':
      case 'M':
        printMenu();
        break;
      case '\n':
      case '\r':
        // Ignore newlines and carriage returns
        break;
      default:
        ESP_LOGI(TAG, "Unknown command: '%c'", cmd);
        printMenu();
        break;
    }
  }
  
  // Brief delay to prevent CPU hogging
  delay(50);
}
