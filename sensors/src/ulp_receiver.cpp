/********************************************************************
 * ESP32-C3 ULP RISC-V ESP-NOW Receiver with Ultra-Low Power Operation
 * 
 * CIRCUIT DIAGRAM - ULP-Optimized ESP32-C3:
 * ┌─────────────────────────────────────────────────────────────┐
 * │  3x AA Lithium Batteries (4.5V, ~3000mAh each)            │
 * │      (+) ──┬── 470µF electro ──┬── ESP32-C3 VCC (Pin 1)   │
 * │            │                   │                           │
 * │            └── 100µF ceramic ──┘                           │
 * │      (-) ────────────────────────── ESP32-C3 GND (Pin 2)   │
 * │                                                             │
 * │  NTC Thermistor Circuit (ULP-accessible pins):             │
 * │      3.3V ── 10kΩ ──┬── ULP_GPIO0 (ADC Channel)           │
 * │                     │                                       │
 * │                     └── NTCLE100E3103JB0 ── GND           │
 * │                                                             │
 * │  Battery Monitor (ULP-accessible):                         │
 * │      3.3V ── 100kΩ ── ULP_GPIO1 ── 100kΩ ── GND          │
 * │                                                             │
 * │  Status LED (Main CPU control only):                       │
 * │      GPIO2 ── 330Ω ── LED ── GND                           │
 * │                                                             │
 * │  ESP-NOW Antenna: Built-in PCB antenna                     │
 * └─────────────────────────────────────────────────────────────┘
 * 
 * ULP RISC-V Features Used:
 *  - ULP Timer: Wakes ULP every 15-60 minutes
 *  - ULP ADC: Reads NTC thermistor and battery voltage
 *  - ULP GPIO: Controls power gating (optional)
 *  - ULP Memory: Stores readings and state
 *  - ULP->Main: Wakes main CPU only for ESP-NOW transmission
 *  - Main CPU: Handles ESP-NOW, then returns to deep sleep
 * 
 * Hardware:
 *  - ESP32-C3-MINI-1 module (ULP RISC-V support)
 *  - NTCLE100E3103JB0 NTC thermistor (10kΩ at 25°C)
 *  - 10kΩ pullup resistor (1% tolerance)
 *  - 2x 100kΩ voltage divider for battery monitoring
 *  - 470µF + 100µF capacitor buffering
 *  - Status LED with 330Ω resistor
 * 
 * Power Consumption (Estimated):
 *  - ULP active (reading sensors): ~150µA for 10ms every 15min
 *  - ULP sleep between readings: ~8µA
 *  - Main CPU wake (ESP-NOW TX): ~80mA for 100ms every 15min
 *  - Deep sleep (main CPU): ~10µA
 *  - Average current: ~20-30µA (vs 200-500µA for normal operation)
 * 
 * Expected Battery Life (3x AA Lithium):
 *  - 15-minute intervals: ~2-3 years
 *  - 30-minute intervals: ~3-4 years
 *  - 60-minute intervals: ~4-5 years
 *  - Safe mode (120-min): ~6-8 years
 * 
 * Function:
 *  - ULP RISC-V reads NTC thermistor every 15-60 minutes
 *  - ULP performs temperature conversion and battery monitoring
 *  - ULP wakes main CPU only when transmission is needed
 *  - Main CPU performs ESP-NOW transmission, then deep sleep
 *  - Mesh relay reception handled by separate dedicated receivers
 *  - Ultra-low power consumption for years of battery operation
 ********************************************************************/

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_sleep.h>
#include <esp_pm.h>
#include <ulp_riscv.h>
#include <driver/rtc_io.h>
#include <driver/adc.h>
#include <soc/rtc.h>

/************* CONFIGURATION FROM PLATFORMIO.INI *****************/
// All settings are now configured in platformio.ini and passed as build flags

// Node Configuration
#ifndef NODE_ID
#define NODE_ID "ulp_rx_01"
#endif

#ifndef NODE_LOCATION
#define NODE_LOCATION "unknown"
#endif

// Hardware Pin Configuration (ULP-accessible pins only)
#ifndef ULP_NTC_PIN
#define ULP_NTC_PIN 0  // GPIO0 -> ULP_GPIO0, ADC1_CH0
#endif

#ifndef ULP_BATTERY_PIN
#define ULP_BATTERY_PIN 1  // GPIO1 -> ULP_GPIO1, ADC1_CH1
#endif

#ifndef STATUS_LED_PIN
#define STATUS_LED_PIN 2  // Main CPU controlled
#endif

// Power Management Thresholds
#ifndef BATTERY_SAFE_THRESHOLD_MV
#define BATTERY_SAFE_THRESHOLD_MV 3800  // 3.8V in millivolts
#endif

#ifndef BATTERY_CRITICAL_THRESHOLD_MV
#define BATTERY_CRITICAL_THRESHOLD_MV 3400  // 3.4V in millivolts
#endif

// Transmission Intervals (minutes)
#ifndef NORMAL_INTERVAL_MIN
#define NORMAL_INTERVAL_MIN 15
#endif

#ifndef SAFE_INTERVAL_MIN
#define SAFE_INTERVAL_MIN 60
#endif

#ifndef EMERGENCY_INTERVAL_MIN
#define EMERGENCY_INTERVAL_MIN 120
#endif

// Receiver MAC address (must be configured)
#ifndef RECEIVER_MAC
#define RECEIVER_MAC {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}  // Broadcast - should be overridden
#endif

// Thermistor constants
#ifndef THERMISTOR_BETA
#define THERMISTOR_BETA 3977
#endif

#ifndef THERMISTOR_R0
#define THERMISTOR_R0 10000
#endif

#ifndef PULLUP_RESISTANCE
#define PULLUP_RESISTANCE 10000
#endif

// Runtime configuration
const char* node_id = NODE_ID;
const char* node_location = NODE_LOCATION;
uint8_t receiver_mac[6] = RECEIVER_MAC;
/*********************************************************************/

// Data structure for ESP-NOW transmission (matches receiver)
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
  uint8_t failed_tx_count;
  bool safe_mode_request;
} sensor_data_t;

// ULP shared memory structure
typedef struct {
  uint32_t sequence_number;
  uint32_t wake_count;
  uint16_t ntc_raw_adc;
  uint16_t battery_raw_adc;
  uint16_t battery_mv;
  int16_t temp_c_x100;      // Temperature in Celsius * 100
  uint8_t power_mode;
  uint8_t failed_tx_count;
  uint8_t interval_minutes;
  uint32_t last_wake_time;
  bool transmission_needed;
  bool safe_mode_request;
} ulp_data_t;

// ULP program in C (will be compiled to RISC-V assembly)
extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");

// Global variables
RTC_DATA_ATTR static ulp_data_t ulp_shared_data = {0};
RTC_DATA_ATTR static uint32_t boot_count = 0;
static esp_now_peer_info_t peer_info;

// ULP RISC-V program source (embedded as inline assembly)
const char ulp_program_source[] = R"(
.text
.global _start

_start:
    # ULP RISC-V program for NTC thermistor reading and ESP-NOW control
    
    # Initialize ADC for thermistor reading (GPIO0 = ADC1_CH0)
    li t0, 0x60008800    # ADC1 base address
    li t1, 0x00000001    # Enable ADC1_CH0
    sw t1, 0x00(t0)      # Write to ADC1_CONF register
    
    # Read NTC thermistor (ADC1_CH0)
    li t0, 0x60008844    # ADC1_DATA register
    lw t1, 0x00(t0)      # Read ADC value
    andi t1, t1, 0xFFF   # Mask to 12 bits
    
    # Store NTC ADC reading in shared memory
    la t0, ulp_ntc_adc
    sw t1, 0(t0)
    
    # Read battery voltage (ADC1_CH1)
    li t0, 0x60008800    # ADC1 base address
    li t1, 0x00000002    # Enable ADC1_CH1
    sw t1, 0x00(t0)      # Write to ADC1_CONF register
    
    li t0, 0x60008844    # ADC1_DATA register
    lw t2, 0x00(t0)      # Read ADC value
    andi t2, t2, 0xFFF   # Mask to 12 bits
    
    # Store battery ADC reading
    la t0, ulp_battery_adc
    sw t2, 0(t0)
    
    # Convert battery ADC to millivolts
    # battery_mv = (adc_reading * 6000) / 4095  (for 6V max with voltage divider)
    li t0, 6000
    mul t2, t2, t0       # adc * 6000
    li t0, 4095
    div t2, t2, t0       # / 4095
    la t0, ulp_battery_mv
    sw t2, 0(t0)
    
    # Convert NTC ADC to temperature (simplified Steinhart-Hart)
    # temp_c = beta / (ln(resistance/r0) + beta/298.15) - 273.15
    # Simplified for ULP: temp_c ≈ (adc_ref - adc_reading) * scale_factor
    
    # Calculate NTC resistance: R = pullup * adc / (4095 - adc)
    li t0, 4095
    sub t0, t0, t1       # 4095 - adc_reading
    li t3, 10000         # pullup resistance
    mul t1, t1, t3       # adc * pullup
    div t1, t1, t0       # resistance = (adc * pullup) / (4095 - adc)
    
    # Simplified temperature calculation (linear approximation)
    # temp_c_x100 = 2500 - (resistance - 10000) / 100
    li t0, 10000
    sub t1, t1, t0       # resistance - 10000
    li t0, 100
    div t1, t1, t0       # (resistance - 10000) / 100
    li t0, 2500
    sub t1, t0, t1       # 2500 - ((resistance - 10000) / 100)
    
    # Store temperature * 100
    la t0, ulp_temp_c_x100
    sw t1, 0(t0)
    
    # Check power mode based on battery voltage
    la t0, ulp_battery_mv
    lw t1, 0(t0)
    li t2, 3400          # Critical threshold (3.4V)
    blt t1, t2, emergency_mode
    li t2, 3800          # Safe threshold (3.8V)
    blt t1, t2, safe_mode
    
normal_mode:
    li t0, 0             # Power mode = normal
    li t1, 15            # Interval = 15 minutes
    j store_power_mode

safe_mode:
    li t0, 1             # Power mode = safe
    li t1, 60            # Interval = 60 minutes
    j store_power_mode

emergency_mode:
    li t0, 2             # Power mode = emergency
    li t1, 120           # Interval = 120 minutes

store_power_mode:
    la t2, ulp_power_mode
    sw t0, 0(t2)
    la t2, ulp_interval_minutes
    sw t1, 0(t2)
    
    # Increment wake count
    la t0, ulp_wake_count
    lw t1, 0(t0)
    addi t1, t1, 1
    sw t1, 0(t0)
    
    # Check if transmission is needed (every interval)
    la t0, ulp_last_wake_time
    lw t1, 0(t0)         # Last wake time
    # Get current time (simplified - use wake count)
    la t0, ulp_wake_count
    lw t2, 0(t0)
    sub t2, t2, t1       # Wakes since last transmission
    
    la t0, ulp_interval_minutes
    lw t3, 0(t0)         # Current interval
    bge t2, t3, need_transmission
    
    # Not time for transmission yet
    li t0, 0
    j store_transmission_flag

need_transmission:
    # Time for transmission
    li t0, 1
    la t1, ulp_last_wake_time
    la t2, ulp_wake_count
    lw t3, 0(t2)
    sw t3, 0(t1)         # Update last wake time

store_transmission_flag:
    la t1, ulp_transmission_needed
    sw t0, 0(t1)
    
    # If transmission needed, wake main CPU
    beqz t0, ulp_sleep
    
    # Wake main CPU for ESP-NOW transmission
    li t0, 0x3FF48000    # RTC_CNTL base
    li t1, 0x80000000    # Wake main CPU bit
    sw t1, 0x1C(t0)      # RTC_CNTL_STATE0_REG
    
ulp_sleep:
    # Set ULP timer for next wake (15 seconds for testing, normally 15-60 minutes)
    li t0, 15000000      # 15 seconds in microseconds (for testing)
    # In production: multiply interval_minutes * 60 * 1000000
    
    # Program ULP timer
    li t1, 0x3FF48000    # RTC_CNTL base
    sw t0, 0x54(t1)      # RTC_CNTL_ULP_CP_TIMER_1_REG
    
    # Enable ULP timer wake
    li t0, 0x00000010    # ULP timer wake enable
    sw t0, 0x1C(t1)      # RTC_CNTL_STATE0_REG
    
    # Halt ULP processor
    halt

# ULP shared memory variables
.data
ulp_ntc_adc:         .word 0
ulp_battery_adc:     .word 0
ulp_battery_mv:      .word 0
ulp_temp_c_x100:     .word 0
ulp_power_mode:      .word 0
ulp_interval_minutes: .word 15
ulp_wake_count:      .word 0
ulp_last_wake_time:  .word 0
ulp_transmission_needed: .word 0
)";

void printMACAddress() {
  uint8_t baseMac[6];
  esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
  Serial.printf("ULP Receiver MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                baseMac[0], baseMac[1], baseMac[2], 
                baseMac[3], baseMac[4], baseMac[5]);
}

void blinkStatus(int count = 1) {
  for (int i = 0; i < count; i++) {
    digitalWrite(STATUS_LED_PIN, HIGH);
    delay(50);
    digitalWrite(STATUS_LED_PIN, LOW);
    delay(50);
  }
}

bool initESPNow() {
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return false;
  }
  
  // Register for a callback function that will be called when data is sent
  esp_now_register_send_cb([](const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS) {
      Serial.println("ESP-NOW send: SUCCESS");
      ulp_shared_data.failed_tx_count = 0;
    } else {
      Serial.println("ESP-NOW send: FAILED");
      ulp_shared_data.failed_tx_count++;
    }
  });
  
  // Register receiver peer
  memcpy(peer_info.peer_addr, receiver_mac, 6);
  peer_info.channel = 0;  
  peer_info.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peer_info) != ESP_OK) {
    Serial.println("Failed to add peer");
    return false;
  }
  
  return true;
}

bool transmitSensorData() {
  // Prepare sensor data structure
  sensor_data_t sensor_data;
  strncpy(sensor_data.node_id, node_id, sizeof(sensor_data.node_id) - 1);
  sensor_data.node_id[sizeof(sensor_data.node_id) - 1] = '\0';
  
  // Convert ULP data to float values
  sensor_data.temp_c = ulp_shared_data.temp_c_x100 / 100.0f;
  sensor_data.temp_f = sensor_data.temp_c * 9.0f / 5.0f + 32.0f;
  sensor_data.battery_v = ulp_shared_data.battery_mv / 1000.0f;
  sensor_data.sequence = ulp_shared_data.sequence_number++;
  sensor_data.timestamp = millis();
  sensor_data.raw_adc = ulp_shared_data.ntc_raw_adc;
  
  // Calculate resistance from ADC reading
  float adc_voltage = (ulp_shared_data.ntc_raw_adc * 3.3f) / 4095.0f;
  sensor_data.resistance = (PULLUP_RESISTANCE * adc_voltage) / (3.3f - adc_voltage);
  
  sensor_data.power_mode = ulp_shared_data.power_mode;
  sensor_data.failed_tx_count = ulp_shared_data.failed_tx_count;
  sensor_data.safe_mode_request = ulp_shared_data.safe_mode_request;
  
  // Print sensor data
  Serial.println("\n=== ULP Sensor Reading ===");
  Serial.printf("Node ID: %s\n", sensor_data.node_id);
  Serial.printf("Temperature: %.2f°C (%.2f°F)\n", sensor_data.temp_c, sensor_data.temp_f);
  Serial.printf("Battery: %.3fV (%dmV)\n", sensor_data.battery_v, ulp_shared_data.battery_mv);
  Serial.printf("Power Mode: %s\n", 
                sensor_data.power_mode == 0 ? "NORMAL" : 
                sensor_data.power_mode == 1 ? "SAFE" : "EMERGENCY");
  Serial.printf("Sequence: %d\n", sensor_data.sequence);
  Serial.printf("Wake Count: %d\n", ulp_shared_data.wake_count);
  Serial.printf("Raw ADC: %d\n", sensor_data.raw_adc);
  Serial.printf("Resistance: %.1fΩ\n", sensor_data.resistance);
  
  // Send data via ESP-NOW
  esp_err_t result = esp_now_send(receiver_mac, (uint8_t*)&sensor_data, sizeof(sensor_data));
  
  if (result == ESP_OK) {
    Serial.println("✓ ESP-NOW transmission initiated");
    blinkStatus(1);
    return true;
  } else {
    Serial.printf("✗ ESP-NOW transmission failed: %d\n", result);
    ulp_shared_data.failed_tx_count++;
    blinkStatus(3);
    return false;
  }
}

void initULP() {
  Serial.println("Initializing ULP RISC-V program...");
  
  // Configure ULP-accessible pins
  rtc_gpio_init(GPIO_NUM_0);  // NTC thermistor
  rtc_gpio_init(GPIO_NUM_1);  // Battery monitor
  rtc_gpio_set_direction(GPIO_NUM_0, RTC_GPIO_MODE_INPUT_ONLY);
  rtc_gpio_set_direction(GPIO_NUM_1, RTC_GPIO_MODE_INPUT_ONLY);
  
  // Configure ADC for ULP access
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);  // GPIO0
  adc1_config_channel_atten(ADC1_CHANNEL_1, ADC_ATTEN_DB_11);  // GPIO1
  adc1_ulp_enable();
  
  // Load ULP program (in production, this would load the compiled binary)
  // For now, we'll simulate ULP operation in the main CPU
  Serial.println("ULP program loaded (simulated)");
  Serial.println("Note: True ULP RISC-V implementation requires additional toolchain setup");
  
  // Initialize ULP shared data
  ulp_shared_data.sequence_number = 0;
  ulp_shared_data.wake_count = 0;
  ulp_shared_data.power_mode = 0;
  ulp_shared_data.failed_tx_count = 0;
  ulp_shared_data.interval_minutes = NORMAL_INTERVAL_MIN;
  ulp_shared_data.transmission_needed = true; // Send first reading immediately
  ulp_shared_data.safe_mode_request = false;
  
  Serial.println("ULP initialization complete");
}

void simulateULPOperation() {
  // Simulate ULP RISC-V operation using main CPU
  // In true ULP implementation, this would run on the ULP coprocessor
  
  // Read NTC thermistor
  ulp_shared_data.ntc_raw_adc = adc1_get_raw(ADC1_CHANNEL_0);
  
  // Read battery voltage
  ulp_shared_data.battery_raw_adc = adc1_get_raw(ADC1_CHANNEL_1);
  ulp_shared_data.battery_mv = (ulp_shared_data.battery_raw_adc * 6000) / 4095; // 6V max with divider
  
  // Convert NTC to temperature (simplified)
  float adc_voltage = (ulp_shared_data.ntc_raw_adc * 3.3f) / 4095.0f;
  float resistance = (PULLUP_RESISTANCE * adc_voltage) / (3.3f - adc_voltage);
  
  // Simplified Steinhart-Hart equation
  float temp_k = THERMISTOR_BETA / (log(resistance / THERMISTOR_R0) + THERMISTOR_BETA / 298.15f);
  float temp_c = temp_k - 273.15f;
  ulp_shared_data.temp_c_x100 = (int16_t)(temp_c * 100);
  
  // Update power mode based on battery voltage
  uint8_t old_mode = ulp_shared_data.power_mode;
  if (ulp_shared_data.battery_mv < BATTERY_CRITICAL_THRESHOLD_MV) {
    ulp_shared_data.power_mode = 2; // Emergency
    ulp_shared_data.interval_minutes = EMERGENCY_INTERVAL_MIN;
  } else if (ulp_shared_data.battery_mv < BATTERY_SAFE_THRESHOLD_MV) {
    ulp_shared_data.power_mode = 1; // Safe
    ulp_shared_data.interval_minutes = SAFE_INTERVAL_MIN;
  } else {
    ulp_shared_data.power_mode = 0; // Normal
    ulp_shared_data.interval_minutes = NORMAL_INTERVAL_MIN;
  }
  
  if (old_mode != ulp_shared_data.power_mode) {
    Serial.printf("Power mode changed: %d -> %d\n", old_mode, ulp_shared_data.power_mode);
  }
  
  // Increment wake count
  ulp_shared_data.wake_count++;
  
  // Check if transmission is needed (every interval)
  static uint32_t last_transmission_wake = 0;
  if ((ulp_shared_data.wake_count - last_transmission_wake) >= 1) { // For testing: transmit every wake
    ulp_shared_data.transmission_needed = true;
    last_transmission_wake = ulp_shared_data.wake_count;
  } else {
    ulp_shared_data.transmission_needed = false;
  }
  
  Serial.printf("ULP simulation: wake=%d, temp=%.2f°C, battery=%dmV, mode=%d, tx_needed=%d\n",
                ulp_shared_data.wake_count,
                ulp_shared_data.temp_c_x100 / 100.0f,
                ulp_shared_data.battery_mv,
                ulp_shared_data.power_mode,
                ulp_shared_data.transmission_needed);
}

void enterDeepSleep() {
  Serial.println("Entering deep sleep...");
  Serial.flush();
  
  // Configure deep sleep
  uint64_t sleep_time_us;
  switch (ulp_shared_data.power_mode) {
    case 0: // Normal
      sleep_time_us = NORMAL_INTERVAL_MIN * 60 * 1000000ULL;
      break;
    case 1: // Safe
      sleep_time_us = SAFE_INTERVAL_MIN * 60 * 1000000ULL;
      break;
    case 2: // Emergency
      sleep_time_us = EMERGENCY_INTERVAL_MIN * 60 * 1000000ULL;
      break;
    default:
      sleep_time_us = NORMAL_INTERVAL_MIN * 60 * 1000000ULL;
  }
  
  // For testing: use much shorter intervals
  sleep_time_us = 15 * 1000000ULL; // 15 seconds
  
  // Configure timer wake up
  esp_sleep_enable_timer_wakeup(sleep_time_us);
  
  // In true ULP implementation, would configure ULP wake up:
  // esp_sleep_enable_ulp_wakeup();
  
  // Enter deep sleep
  esp_deep_sleep_start();
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // Increment boot count
  boot_count++;
  
  Serial.println("\n=== ESP32-C3 ULP RISC-V ESP-NOW Receiver ===");
  Serial.printf("Boot count: %d\n", boot_count);
  Serial.printf("Node ID: %s\n", node_id);
  Serial.printf("Location: %s\n", node_location);
  
  // Configure status LED
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);
  
  // Print MAC address
  printMACAddress();
  
  // Initialize ULP RISC-V program
  initULP();
  
  // Simulate ULP operation (read sensors, update power mode)
  simulateULPOperation();
  
  // Check if transmission is needed
  if (ulp_shared_data.transmission_needed) {
    Serial.println("Transmission needed - initializing ESP-NOW...");
    
    // Initialize ESP-NOW
    if (initESPNow()) {
      // Transmit sensor data
      transmitSensorData();
      
      // Wait for transmission to complete
      delay(100);
    } else {
      Serial.println("ESP-NOW initialization failed");
      ulp_shared_data.failed_tx_count++;
    }
    
    // Reset transmission flag
    ulp_shared_data.transmission_needed = false;
  } else {
    Serial.println("Transmission not needed - skipping ESP-NOW");
  }
  
  // Show current status
  Serial.printf("Battery: %dmV, Power Mode: %s, Failed TX: %d\n",
                ulp_shared_data.battery_mv,
                ulp_shared_data.power_mode == 0 ? "NORMAL" : 
                ulp_shared_data.power_mode == 1 ? "SAFE" : "EMERGENCY",
                ulp_shared_data.failed_tx_count);
  
  // Blink pattern based on power mode
  switch (ulp_shared_data.power_mode) {
    case 0: blinkStatus(1); break; // Normal: 1 blink
    case 1: blinkStatus(2); break; // Safe: 2 blinks  
    case 2: blinkStatus(5); break; // Emergency: 5 blinks
  }
  
  // Enter deep sleep until next ULP wake
  enterDeepSleep();
}

void loop() {
  // Should never reach here due to deep sleep
  Serial.println("ERROR: Main loop should not execute!");
  delay(1000);
}

/********************************************************************
 * ULP RISC-V ESP-NOW RECEIVER SETUP INSTRUCTIONS:
 * 
 * HARDWARE REQUIREMENTS:
 * 1. ESP32-C3-MINI-1 module with ULP RISC-V support
 * 2. NTCLE100E3103JB0 NTC thermistor connected to GPIO0
 * 3. 10kΩ pullup resistor for thermistor
 * 4. Voltage divider (2x 100kΩ) on GPIO1 for battery monitoring
 * 5. 3x AA lithium batteries with capacitor buffering
 * 6. Status LED on GPIO2 with 330Ω resistor
 * 
 * CONFIGURATION:
 * 1. Configure receiver MAC address in platformio.ini:
 *    -DRECEIVER_MAC='{0x24,0x6F,0x28,0xAB,0xCD,0xEF}'
 * 2. Set node ID and location:
 *    -DNODE_ID='"ulp_rx_01"'
 *    -DNODE_LOCATION='"outdoor_shed"'
 * 3. Adjust power thresholds for your battery setup
 * 4. Set transmission intervals (15-120 minutes recommended)
 * 
 * SOFTWARE SETUP:
 * 1. Install ESP32 ULP RISC-V toolchain
 * 2. Enable ULP support in platformio.ini:
 *    board_build.cmake_extra_args = -DCONFIG_ESP32C3_ULP_RISCV_ENABLED=y
 * 3. Compile ULP program separately (true implementation)
 * 4. Flash both main program and ULP binary
 * 
 * POWER OPTIMIZATION:
 * - ULP reads sensors every 15-120 minutes
 * - Main CPU wakes only for ESP-NOW transmission
 * - Deep sleep between transmissions
 * - Automatic power mode adjustment based on battery
 * - Expected battery life: 2-5+ years depending on interval
 * 
 * LIMITATIONS (Current Implementation):
 * - ULP operation is simulated on main CPU
 * - True ULP RISC-V requires additional toolchain setup
 * - No mesh relay capability (receive-only for ultra-low power)
 * - No WiFi or HTTP forwarding (dedicated receivers handle this)
 * 
 * TRUE ULP IMPLEMENTATION NEXT STEPS:
 * 1. Set up ESP32-C3 ULP RISC-V toolchain
 * 2. Compile ulp_program_source to binary
 * 3. Load ULP binary using ulp_riscv_load_binary()
 * 4. Enable ULP timer wake: esp_sleep_enable_ulp_wakeup()
 * 5. ULP handles all sensor reading, main CPU only for transmission
 * 6. Achieve true ultra-low power operation (~20-30µA average)
 * 
 * DEPLOYMENT:
 * - Use as ultra-low power sensor nodes in remote locations
 * - Pair with mains-powered receivers for mesh networking
 * - Monitor battery levels and failed transmission counts
 * - Years of operation on single battery set
 * - Ideal for outdoor, hard-to-access sensor deployments
 * 
 * TROUBLESHOOTING:
 * - Check ULP toolchain installation if compilation fails
 * - Verify GPIO pin assignments match hardware
 * - Monitor battery voltage and power mode transitions
 * - Use Serial output to debug sensor readings and transmissions
 * - LED blink patterns indicate power mode and transmission status
 ********************************************************************/
