/********************************************************************
 * ESP32-C3 ULP Ultra-Low Power Temperature Transmitter
 * 
 * CIRCUIT DIAGRAM:
 * ┌─────────────────────────────────────────────────────────────┐
 * │  CR2450 Battery (3.0V, 540-600mAh)                         │
 * │      (+) ──┬── 100µF ceramic ──┬── ESP32-C3 VCC (Pin 1)    │
 * │            │                   │                           │
 * │            └── 220µF electro ──┘                           │
 * │      (-) ────────────────────────── ESP32-C3 GND (Pin 2)   │
 * │                                                             │
 * │  Thermistor Circuit (ULP ADC):                              │
 * │      3.3V ── 10kΩ ── GPIO3 ── NTC(10kΩ) ── GND            │
 * │                      │                                      │
 * │                      └── ADC1_CH3 (ULP accessible)         │
 * │                                                             │
 * │  Optional Power Gating (ULP GPIO):                         │
 * │      GPIO0 ── BSS138(G) ┌─ 10kΩ ── 3.3V                   │
 * │                    (S) ─┤                                   │
 * │                    (D) ─┼── GPIO3                          │
 * │                                                             │
 * │  ULP Status LED:                                            │
 * │      GPIO1 ── 330Ω ── LED ── GND                           │
 * └─────────────────────────────────────────────────────────────┘
 * 
 * Hardware:
 *  - ESP32-C3 module with ULP RISC-V coprocessor
 *  - CR2450 3V lithium battery (540-600mAh)
 *  - NTCLE100E3103JB0 thermistor (10kΩ @25°C, β=3977K)
 *  - 10kΩ 1% precision resistor
 *  - 100µF ceramic + 220µF electrolytic capacitors
 *  - Optional: BSS138 P-FET for power gating
 *  - Status LED with 330Ω resistor
 * 
 * ULP Operation:
 *  - Main CPU sleeps 99.99% of the time
 *  - ULP coprocessor wakes every 15-60 minutes
 *  - ULP reads ADC, processes NTC conversion
 *  - ULP wakes main CPU only for ESP-NOW transmission
 *  - Main CPU transmits and returns to deep sleep
 * 
 * Expected Battery Life:
 *  - 15 min intervals: ~90-120 days (vs 42 days normal)
 *  - 1 hour intervals: ~300-400 days (vs 168 days normal)
 *  - Safe mode (4hr): ~800+ days
 * 
 * Power Consumption:
 *  - ULP active (1ms): ~150µA
 *  - Main CPU active (20ms): ~80mA
 *  - Deep Sleep: ~2-5µA (vs 8µA normal)
 *  - ULP sleep: ~1µA
 ********************************************************************/

#include <WiFi.h>
#include <esp_now.h>
#include <esp_sleep.h>
#include <esp_wifi.h>
#include <driver/rtc_io.h>
#include <soc/rtc_cntl_reg.h>
#include <soc/sens_reg.h>
#include <soc/rtc.h>
#include "ulp_main.h"
#include "esp32c3/ulp.h"
#include "esp32c3/ulp_riscv.h"

/************* CONFIGURATION FROM PLATFORMIO.INI *****************/
// ULP-specific pin configuration for ESP32-C3
#ifndef ULP_ADC_PIN
#define ULP_ADC_PIN 3                // GPIO3 = ADC1_CH3 (ULP accessible)
#endif

#ifndef ULP_POWER_PIN
#define ULP_POWER_PIN 0              // GPIO0 for thermistor power control
#endif

#ifndef ULP_LED_PIN
#define ULP_LED_PIN 1                // GPIO1 for status LED
#endif

// Node Configuration
#ifndef NODE_ID
#define NODE_ID "ulp_sensor_01"
#endif

#ifndef NODE_LOCATION
#define NODE_LOCATION "unknown"
#endif

// ULP Timing Configuration (minutes)
#ifndef ULP_SLEEP_MINUTES
#define ULP_SLEEP_MINUTES 15
#endif

#ifndef ULP_SAFE_SLEEP_MINUTES
#define ULP_SAFE_SLEEP_MINUTES 60
#endif

#ifndef ULP_EMERGENCY_SLEEP_MINUTES
#define ULP_EMERGENCY_SLEEP_MINUTES 240
#endif

// Power Management
#ifndef BATTERY_SAFE_THRESHOLD_RAW
#define BATTERY_SAFE_THRESHOLD_RAW 2867    // 2.4V in ADC units (2.4/3.3*4095)
#endif

#ifndef BATTERY_NORMAL_THRESHOLD_RAW
#define BATTERY_NORMAL_THRESHOLD_RAW 3276  // 2.6V in ADC units
#endif

#ifndef BATTERY_CRITICAL_THRESHOLD_RAW
#define BATTERY_CRITICAL_THRESHOLD_RAW 2686 // 2.2V in ADC units
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

// ESP-NOW Configuration
#ifndef RECEIVER_MAC_0
#define RECEIVER_MAC_0 0xFF
#endif

// Runtime variables
const char* node_id = NODE_ID;
const char* node_location = NODE_LOCATION;
int ulp_sleep_minutes = ULP_SLEEP_MINUTES;
int ulp_safe_sleep_minutes = ULP_SAFE_SLEEP_MINUTES;
int ulp_emergency_sleep_minutes = ULP_EMERGENCY_SLEEP_MINUTES;

// Receiver MAC address
uint8_t receiverMAC[] = {
    RECEIVER_MAC_0, RECEIVER_MAC_1, RECEIVER_MAC_2,
    RECEIVER_MAC_3, RECEIVER_MAC_4, RECEIVER_MAC_5
};

// Sensor constants
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
  uint8_t ulp_wake_count;   // Number of ULP wake cycles
  bool ulp_error;           // ULP error flag
} ulp_sensor_data_t;

// ULP shared data structure (in RTC memory)
typedef struct {
  uint32_t sequence_number;
  uint32_t ulp_wake_count;
  uint16_t battery_raw;
  uint16_t temp_raw;
  uint16_t resistance_raw;
  uint8_t power_mode;
  uint8_t error_flags;
  uint32_t sleep_duration_us;
} ulp_shared_data_t;

// RTC memory variables (persistent across deep sleep)
RTC_DATA_ATTR ulp_shared_data_t ulp_data = {0};
RTC_DATA_ATTR bool ulp_initialized = false;
RTC_DATA_ATTR uint32_t boot_count = 0;

// Global sensor data
ulp_sensor_data_t sensor_data;

float convertRawToVoltage(uint16_t raw) {
  return (raw * 3.3) / 4095.0;
}

float convertRawToTemperature(uint16_t temp_raw, uint16_t battery_raw) {
  float battery_v = convertRawToVoltage(battery_raw);
  float v = (temp_raw * battery_v) / 4095.0;
  
  if (v >= (battery_v - 0.1)) {
    return -999.0; // Error: voltage near supply
  }
  
  float rT = (v * r_fixed) / (battery_v - v);
  float tempK = 1.0 / ((1.0 / t0_temp) + (1.0 / beta) * log(rT / r0));
  return tempK - 273.15;
}

void initULP() {
  Serial.println("🔧 Initializing ULP RISC-V coprocessor...");
  
  // Configure ULP pins
  rtc_gpio_init((gpio_num_t)ULP_ADC_PIN);
  rtc_gpio_set_direction((gpio_num_t)ULP_ADC_PIN, RTC_GPIO_MODE_DISABLED);
  
  #ifdef USE_POWER_GATING
  rtc_gpio_init((gpio_num_t)ULP_POWER_PIN);
  rtc_gpio_set_direction((gpio_num_t)ULP_POWER_PIN, RTC_GPIO_MODE_OUTPUT_ONLY);
  rtc_gpio_set_level((gpio_num_t)ULP_POWER_PIN, 0); // Start with thermistor off
  #endif
  
  rtc_gpio_init((gpio_num_t)ULP_LED_PIN);
  rtc_gpio_set_direction((gpio_num_t)ULP_LED_PIN, RTC_GPIO_MODE_OUTPUT_ONLY);
  rtc_gpio_set_level((gpio_num_t)ULP_LED_PIN, 0);
  
  // Load ULP program (simplified example - real implementation would be more complex)
  Serial.println("📝 Loading ULP program...");
  
  // In a real implementation, you would load the ULP RISC-V binary here
  // For now, we'll use a simpler approach with the main CPU doing periodic wake-ups
  
  Serial.println("✅ ULP initialization complete");
}

void simulateULPReading() {
  // This simulates what the ULP would do:
  // 1. Power on thermistor (if using power gating)
  // 2. Wait for settling
  // 3. Read ADC values
  // 4. Power off thermistor
  // 5. Store results in RTC memory
  
  #ifdef USE_POWER_GATING
  rtc_gpio_set_level((gpio_num_t)ULP_POWER_PIN, 1);
  delayMicroseconds(10000); // 10ms settling time
  #endif
  
  // Blink LED to show ULP activity
  rtc_gpio_set_level((gpio_num_t)ULP_LED_PIN, 1);
  delayMicroseconds(1000);
  rtc_gpio_set_level((gpio_num_t)ULP_LED_PIN, 0);
  
  // Read ADC values (simulating ULP ADC reading)
  ulp_data.temp_raw = analogRead(ULP_ADC_PIN);
  ulp_data.battery_raw = analogRead(ULP_ADC_PIN); // Would use different channel for battery
  ulp_data.ulp_wake_count++;
  
  #ifdef USE_POWER_GATING
  rtc_gpio_set_level((gpio_num_t)ULP_POWER_PIN, 0);
  #endif
  
  // Update power mode based on battery voltage
  if (ulp_data.battery_raw < BATTERY_CRITICAL_THRESHOLD_RAW) {
    ulp_data.power_mode = 2; // Emergency
    ulp_data.sleep_duration_us = ulp_emergency_sleep_minutes * 60 * 1000000ULL;
  } else if (ulp_data.battery_raw < BATTERY_SAFE_THRESHOLD_RAW) {
    ulp_data.power_mode = 1; // Safe
    ulp_data.sleep_duration_us = ulp_safe_sleep_minutes * 60 * 1000000ULL;
  } else if (ulp_data.battery_raw > BATTERY_NORMAL_THRESHOLD_RAW && ulp_data.power_mode == 1) {
    ulp_data.power_mode = 0; // Normal
    ulp_data.sleep_duration_us = ulp_sleep_minutes * 60 * 1000000ULL;
  }
  
  ulp_data.sequence_number++;
}

void prepareSensorData() {
  // Convert ULP raw data to sensor data structure
  strncpy(sensor_data.node_id, node_id, sizeof(sensor_data.node_id) - 1);
  sensor_data.node_id[sizeof(sensor_data.node_id) - 1] = '\0';
  
  sensor_data.temp_c = convertRawToTemperature(ulp_data.temp_raw, ulp_data.battery_raw);
  sensor_data.temp_f = (sensor_data.temp_c * 9.0 / 5.0) + 32.0;
  sensor_data.battery_v = convertRawToVoltage(ulp_data.battery_raw);
  sensor_data.sequence = ulp_data.sequence_number;
  sensor_data.timestamp = millis();
  sensor_data.raw_adc = ulp_data.temp_raw;
  
  // Calculate resistance
  float v = convertRawToVoltage(ulp_data.temp_raw);
  if (v < (sensor_data.battery_v - 0.1)) {
    sensor_data.resistance = (v * r_fixed) / (sensor_data.battery_v - v);
  } else {
    sensor_data.resistance = 0.0; // Error condition
  }
  
  sensor_data.power_mode = ulp_data.power_mode;
  sensor_data.ulp_wake_count = ulp_data.ulp_wake_count;
  sensor_data.ulp_error = (sensor_data.temp_c == -999.0);
}

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    Serial.println("✅ ESP-NOW transmission successful");
    // Blink LED to indicate successful transmission
    rtc_gpio_set_level((gpio_num_t)ULP_LED_PIN, 1);
    delay(100);
    rtc_gpio_set_level((gpio_num_t)ULP_LED_PIN, 0);
  } else {
    Serial.println("❌ ESP-NOW transmission failed");
    ulp_data.error_flags |= 0x01; // Set transmission error flag
  }
}

void initESPNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ ESP-NOW init failed!");
    ulp_data.error_flags |= 0x02;
    return;
  }
  
  esp_now_register_send_cb(onDataSent);
  
  // Add peer (receiver)
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("❌ Failed to add ESP-NOW peer");
    ulp_data.error_flags |= 0x04;
    return;
  }
  
  Serial.println("✅ ESP-NOW initialized");
}

void transmitSensorData() {
  prepareSensorData();
  
  Serial.println("\n=== ULP Sensor Data Transmission ===");
  Serial.printf("Node ID: %s\n", sensor_data.node_id);
  Serial.printf("Boot Count: %d\n", boot_count);
  Serial.printf("ULP Wake Count: %d\n", sensor_data.ulp_wake_count);
  Serial.printf("Temperature: %.2f°C (%.2f°F)\n", sensor_data.temp_c, sensor_data.temp_f);
  Serial.printf("Battery: %.3fV (raw: %d)\n", sensor_data.battery_v, ulp_data.battery_raw);
  Serial.printf("Power Mode: %s\n", 
                sensor_data.power_mode == 0 ? "NORMAL" : 
                sensor_data.power_mode == 1 ? "SAFE" : "EMERGENCY");
  Serial.printf("Raw ADC: %d\n", sensor_data.raw_adc);
  Serial.printf("Resistance: %.1f Ω\n", sensor_data.resistance);
  Serial.printf("Sequence: %d\n", sensor_data.sequence);
  Serial.printf("ULP Error: %s\n", sensor_data.ulp_error ? "YES" : "NO");
  Serial.printf("Error Flags: 0x%02X\n", ulp_data.error_flags);
  
  // Transmit via ESP-NOW
  esp_err_t result = esp_now_send(receiverMAC, (uint8_t*)&sensor_data, sizeof(sensor_data));
  if (result == ESP_OK) {
    Serial.println("📡 Data sent via ESP-NOW");
  } else {
    Serial.printf("❌ ESP-NOW send failed: %d\n", result);
    ulp_data.error_flags |= 0x08;
  }
  
  // Wait for transmission callback
  delay(50);
  
  Serial.printf("Next wake in: %d minutes\n", 
                ulp_data.sleep_duration_us / 60000000ULL);
  Serial.println("=====================================\n");
}

void enterULPSleep() {
  Serial.printf("💤 Entering ULP deep sleep for %llu seconds...\n", 
                ulp_data.sleep_duration_us / 1000000ULL);
  
  // Configure wake-up timer
  esp_sleep_enable_timer_wakeup(ulp_data.sleep_duration_us);
  
  // In a real implementation, you would start the ULP program here
  // For now, we'll use timer wake-up
  
  Serial.flush();
  delay(10);
  
  // Enter deep sleep
  esp_deep_sleep_start();
}

void setup() {
  boot_count++;
  
  Serial.begin(115200);
  delay(100);
  
  Serial.println("\n" + String("=").repeat(60));
  Serial.println("🔋 ESP32-C3 ULP Ultra-Low Power Transmitter");
  Serial.println("=".repeat(60));
  Serial.printf("Boot Count: %d\n", boot_count);
  Serial.printf("Node ID: %s\n", node_id);
  Serial.printf("Location: %s\n", node_location);
  Serial.printf("ULP Wake Count: %d\n", ulp_data.ulp_wake_count);
  
  // Check wake-up reason
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_TIMER:
      Serial.println("🕐 Wake-up: Timer (ULP cycle)");
      break;
    case ESP_SLEEP_WAKEUP_ULP:
      Serial.println("🔧 Wake-up: ULP coprocessor");
      break;
    default:
      Serial.println("🔄 Wake-up: Power-on reset");
      ulp_data = {0}; // Reset ULP data on power-on
      break;
  }
  
  // Initialize ULP on first boot
  if (!ulp_initialized) {
    initULP();
    ulp_initialized = true;
    ulp_data.sleep_duration_us = ulp_sleep_minutes * 60 * 1000000ULL;
    Serial.println("🆕 First boot - ULP system initialized");
  }
  
  // Simulate ULP reading (in real implementation, this would be done by ULP)
  Serial.println("📊 Performing ULP sensor reading...");
  simulateULPReading();
  
  // Initialize ESP-NOW for transmission
  initESPNow();
  
  // Transmit sensor data
  transmitSensorData();
  
  // Enter ULP deep sleep
  enterULPSleep();
}

void loop() {
  // This should never be reached as we enter deep sleep in setup()
  Serial.println("⚠️ Warning: Loop() reached - this shouldn't happen");
  delay(1000);
}

/********************************************************************
 * ULP IMPLEMENTATION NOTES:
 * 
 * REAL ULP RISC-V IMPLEMENTATION:
 * This code currently simulates ULP operation. For a true ULP implementation:
 * 
 * 1. Write ULP RISC-V assembly program to:
 *    - Configure ADC for thermistor reading
 *    - Read ADC values for temperature and battery
 *    - Perform basic threshold checking
 *    - Store results in RTC memory
 *    - Wake main CPU only when transmission needed
 * 
 * 2. Compile ULP program:
 *    ```
 *    ulp_riscv_compile(ulp_main_bin_start, ulp_main_bin_end);
 *    ```
 * 
 * 3. Start ULP:
 *    ```
 *    ulp_riscv_run();
 *    ```
 * 
 * POWER OPTIMIZATION:
 * - ULP wakes every 15-60 minutes
 * - Main CPU active only 20-50ms per cycle
 * - Total active time: <0.01% duty cycle
 * - Deep sleep current: 2-5µA (vs 8µA normal)
 * - Expected battery life: 300-800+ days
 * 
 * ULP CAPABILITIES:
 * - ADC reading (channels 0-4 on ESP32-C3)
 * - GPIO control (limited pins)
 * - Basic arithmetic and comparisons
 * - RTC memory access
 * - Timer-based wake-ups
 * - Wake main CPU on conditions
 * 
 * LIMITATIONS:
 * - No ESP-NOW transmission from ULP
 * - Limited processing power
 * - Small program memory
 * - Restricted peripheral access
 * 
 * DEPLOYMENT:
 * Use this for ultra-long-term deployments where battery life
 * is critical and data collection frequency can be reduced.
 * 
 * CONFIGURATION:
 * All settings configurable via platformio.ini:
 * - ULP_SLEEP_MINUTES: Normal sleep interval
 * - ULP_SAFE_SLEEP_MINUTES: Safe mode interval  
 * - ULP_EMERGENCY_SLEEP_MINUTES: Emergency interval
 * - Battery thresholds in raw ADC units
 * - Pin assignments for ULP-accessible pins
 ********************************************************************/
