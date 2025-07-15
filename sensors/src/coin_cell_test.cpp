#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <driver/adc.h>
#include <esp_sleep.h>
#include <esp_wifi.h>
#include <esp_bt.h>
#include "esp_log.h"

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
 * NTC Thermistor Circuit:
 *    3V3 --- 10kΩ Resistor ---+--- GPIO0 (ADC1_CH0)
 *                             |
 *                             +--- 10kΩ NTC Thermistor --- GND
 * 
 * Battery Monitoring Circuit:
 *    3V3 --- 100kΩ Resistor ---+--- GPIO1 (ADC1_CH1)
 *                              |
 *                              +--- 100kΩ Resistor --- GND
 * 
 * Power Saving Features:
 * - CPU frequency reduced to 10 MHz when not transmitting
 * - WiFi in power-save mode when not actively transmitting
 * - Bluetooth disabled
 * - Deep sleep between measurements
 */

// ESP logging tag - minimal logging in production
static const char *TAG = "COIN_CELL_TEST";

// Configuration
#define SENSOR_PIN       0      // GPIO0 - ADC1_CH0
#define BATTERY_PIN      1      // GPIO1 - ADC1_CH1 
#define SLEEP_TIME_US    300000000  // 5 minutes between readings (5 * 60 * 1000000)

// Power management functions
void disableUnusedPeripherals() {
  // Disable Bluetooth
  ESP_LOGI(TAG, "Disabling Bluetooth");
  esp_err_t bt_result = esp_bt_controller_disable();
  if (bt_result != ESP_OK) {
    ESP_LOGI(TAG, "Error disabling Bluetooth: %d", bt_result);
  }
  
  // Set WiFi to minimum power
  ESP_LOGI(TAG, "Setting WiFi to minimum power mode");
  esp_err_t wifi_result = esp_wifi_set_ps(WIFI_PS_MAX_MODEM);
  if (wifi_result != ESP_OK) {
    ESP_LOGI(TAG, "Error setting WiFi power mode: %d", wifi_result);
  }
  
  // We'll avoid configuring any GPIO pins for now
  ESP_LOGI(TAG, "Skipping GPIO configuration");
}

void setCpuFrequencyLow() {
  // Set CPU frequency to lowest possible (10MHz)
  setCpuFrequencyMhz(10);
}

void setCpuFrequencyNormal() {
  // Set CPU frequency back to normal (80MHz)
  setCpuFrequencyMhz(80);
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

// ESP-NOW data structure
typedef struct {
  uint32_t node_id;               // Node identifier
  uint16_t raw_temp_adc;          // Raw ADC value from thermistor
  uint16_t battery_mv;            // Battery voltage in millivolts
  uint32_t counter;               // Simple counter
} sensor_data_t;

// Get battery voltage in mV from ADC reading
uint16_t getBatteryVoltage(uint16_t raw_adc) {
  // For a voltage divider with 2x 100K resistors (1:1 ratio)
  // Full range is 0-3.3V, converted to 0-4095
  // Formula: voltage = (raw_adc * 3300 / 4095) * 2
  // Simplified: voltage = raw_adc * 1.61
  return raw_adc * 1.61;
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
  ESP_LOGI(TAG, "Data received from MAC: %02X:%02X:%02X:%02X:%02X:%02X",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  ESP_LOGI(TAG, "Received %d bytes of data", data_len);
  
  // If data matches our struct, parse and display it
  if (data_len == sizeof(sensor_data_t)) {
    sensor_data_t *receivedData = (sensor_data_t *)data;
    ESP_LOGI(TAG, "Received data - Node ID: 0x%08X, ADC: %d, Battery: %d mV, Counter: %d", 
             receivedData->node_id, receivedData->raw_temp_adc, 
             receivedData->battery_mv, receivedData->counter);
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
  sensor_data_t sensorData;
  
  // Fill data structure
  sensorData.node_id = 0x12345678;  // Example node ID
  sensorData.raw_temp_adc = adc_reading;
  sensorData.battery_mv = getBatteryVoltage(battery_reading);
  sensorData.counter = counter++;  // Increment counter
  
  // Send data
  esp_err_t result = esp_now_send(receiverMac, (uint8_t *)&sensorData, sizeof(sensorData));
  
  if (result == ESP_OK) {
    ESP_LOGI(TAG, "ESP-NOW packet sent successfully (API call OK)");
    ESP_LOGI(TAG, "Packet: node_id=0x%X, adc=%d, battery=%d mV, counter=%d", 
             sensorData.node_id, sensorData.raw_temp_adc, 
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
  // Read thermistor with error checking
  adc_reading = adc1_get_raw(ADC1_CHANNEL_0);
  if (adc_reading == 0) {
    ESP_LOGI(TAG, "Warning: Zero reading from ADC1_CHANNEL_0");
  }
  
  // Read battery with error checking
  battery_reading = adc1_get_raw(ADC1_CHANNEL_1);
  if (battery_reading == 0) {
    ESP_LOGI(TAG, "Warning: Zero reading from ADC1_CHANNEL_1");
  }
  
  ESP_LOGI(TAG, "ADC reading: %d", adc_reading);
  ESP_LOGI(TAG, "Battery reading: %d", battery_reading);
  ESP_LOGI(TAG, "Battery voltage: %d mV", getBatteryVoltage(battery_reading));
}

// Delay between transmissions
void delaySend() {
  ESP_LOGI(TAG, "Waiting 5 seconds before next transmission...");
  delay(5000);
}

void setup() {
  // Initialize serial for debugging
  Serial.begin(115200);
  delay(1000);
  
  ESP_LOGI(TAG, "************ COIN CELL TEST - ULTRA LOW POWER MODE ************");
  
  // First step: Just print MAC address
  uint8_t mac[6];
  esp_efuse_mac_get_default(mac);
  ESP_LOGI(TAG, "MAC Address: %02X:%02X:%02X:%02X:%02X:%02X", 
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  
  // Convert MAC string to byte array
  ESP_LOGI(TAG, "Converting MAC string to bytes");
  macStringToBytes(receiverMacStr, receiverMac);
  
  // Initialize ADC before changing CPU frequency
  ESP_LOGI(TAG, "Setting up ADC...");
  setupADC();
  ESP_LOGI(TAG, "ADC setup complete");
  
  // Take initial readings
  ESP_LOGI(TAG, "Taking initial sensor readings");
  adc_reading = adc1_get_raw(ADC1_CHANNEL_0);
  battery_reading = adc1_get_raw(ADC1_CHANNEL_1);
  ESP_LOGI(TAG, "Initial ADC reading: %d", adc_reading);
  ESP_LOGI(TAG, "Initial Battery reading: %d", battery_reading);
  
  // Now set CPU to low frequency
  ESP_LOGI(TAG, "Setting CPU frequency to low");
  setCpuFrequencyNormal(); // First set to normal to ensure WiFi works
  ESP_LOGI(TAG, "CPU frequency: %d MHz", getCpuFrequencyMhz());
  
  // Set up ESP-NOW
  ESP_LOGI(TAG, "Setting up ESP-NOW");
  setupEspNow();
  ESP_LOGI(TAG, "ESP-NOW setup complete");
  
  // Now that everything is set up, we can lower CPU frequency
  ESP_LOGI(TAG, "Lowering CPU frequency for power saving");
  setCpuFrequencyLow();
  ESP_LOGI(TAG, "Current CPU frequency: %d MHz", getCpuFrequencyMhz());
  
  // Disable unused peripherals to save power
  ESP_LOGI(TAG, "Disabling unused peripherals");
  disableUnusedPeripherals();
  ESP_LOGI(TAG, "Peripherals disabled");
  
  ESP_LOGI(TAG, "Setup complete, will now read sensors and send data every 5 seconds");
}

void loop() {
  // Increase CPU frequency for WiFi operations
  setCpuFrequencyNormal();
  ESP_LOGI(TAG, "CPU frequency increased for communication");
  
  // Read sensor values
  readSensors();
  
  // Make sure WiFi is in station mode for ESP-NOW
  if (WiFi.getMode() != WIFI_STA) {
    ESP_LOGI(TAG, "Setting WiFi to station mode");
    WiFi.mode(WIFI_STA);
    delay(100);
  }
  
  // Check if ESP-NOW is initialized, if not reinitialize it
  ESP_LOGI(TAG, "Checking ESP-NOW initialization");
  esp_err_t status = esp_now_init();
  if (status != ESP_OK) {
    ESP_LOGI(TAG, "ESP-NOW not initialized, reinitializing...");
    setupEspNow();
  } else {
    ESP_LOGI(TAG, "ESP-NOW already initialized");
  }
  
  // Try alternating between broadcast and specific MACs
  static int attempt = 0;
  attempt = (attempt + 1) % 3;
  
  if (attempt == 0) {
    // Try broadcast address
    ESP_LOGI(TAG, "Sending ESP-NOW packet to broadcast MAC");
    macStringToBytes("FF:FF:FF:FF:FF:FF", receiverMac);
  } else if (attempt == 1) {
    // Try USB-powered MAC
    ESP_LOGI(TAG, "Sending ESP-NOW packet to USB-powered MAC");
    macStringToBytes(usbPoweredMacStr, receiverMac);
  } else {
    // Try coin-powered MAC
    ESP_LOGI(TAG, "Sending ESP-NOW packet to coin-powered MAC");
    macStringToBytes(coinPoweredMacStr, receiverMac);
  }
  
  // Send data
  sendEspNowData();
  
  // Wait for transmission to complete
  delay(100);
  
  // Log the status of the last transmission
  if (lastTransmitSuccess) {
    ESP_LOGI(TAG, "Last transmission was successful");
  } else {
    ESP_LOGI(TAG, "Last transmission failed");
  }
  
  // Put WiFi in power save mode instead of turning it off
  // This will save power but keep ESP-NOW working
  ESP_LOGI(TAG, "Setting WiFi to power save mode");
  esp_wifi_set_ps(WIFI_PS_MAX_MODEM);
  
  // Set CPU back to low frequency
  setCpuFrequencyLow();
  ESP_LOGI(TAG, "CPU frequency lowered to save power");
  ESP_LOGI(TAG, "Current CPU frequency: %d MHz", getCpuFrequencyMhz());
  
  // Wait before next transmission
  delaySend();
}
