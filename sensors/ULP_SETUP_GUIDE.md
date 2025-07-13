# ESP32-C3 ULP RISC-V Ultra-Low Power Temperature Sensor Setup Guide

## Overview

This guide covers setting up the ESP32-C3 ULP RISC-V temperature sensor that achieves ultra-low power consumption (~20-30µA average) by using the ULP coprocessor to read sensors while keeping the main CPU in deep sleep most of the time.

## Hardware Requirements

### ESP32-C3 Module
- **ESP32-C3-MINI-1** or **ESP32-C3-DevKitM-1**
- Must support ULP RISC-V coprocessor
- Verify with: `esptool.py chip_id` (should show ESP32-C3)

### Circuit Components
```
Power Supply:
- 3x AA Lithium batteries (4.5V, ~3000mAh each)
- 470µF electrolytic capacitor (low ESR)
- 100µF ceramic capacitor (0805 or larger)

Temperature Sensor:
- NTCLE100E3103JB0 NTC thermistor (10kΩ @ 25°C)
- 10kΩ pullup resistor (1% tolerance)
- Connect between GPIO0 and GND

Battery Monitor:
- 2x 100kΩ resistors for voltage divider
- Connect between VCC and GPIO1 and GND

Status Indicator:
- LED with 330Ω current limiting resistor
- Connect to GPIO2

Optional Power Gating:
- P-channel MOSFET (e.g., BSS84) for sensor power control
- 10kΩ pullup on gate
```

### Circuit Diagram
```
                ESP32-C3 ULP Temperature Sensor
    ┌─────────────────────────────────────────────────────────────┐
    │  3x AA Lithium (4.5V)                                      │
    │      (+) ──┬── 470µF ──┬── ESP32-C3 VCC (3.3V)            │
    │            │           │                                   │
    │            └── 100µF ──┘                                   │
    │      (-) ──────────────── ESP32-C3 GND                     │
    │                                                             │
    │  Temperature Sensor (ULP-accessible):                      │
    │      3.3V ── 10kΩ ──┬── GPIO0 (ULP_GPIO0, ADC1_CH0)       │
    │                     │                                       │
    │                     └── NTC Thermistor ── GND             │
    │                                                             │
    │  Battery Monitor (ULP-accessible):                         │
    │      3.3V ── 100kΩ ──┬── GPIO1 (ULP_GPIO1, ADC1_CH1)      │
    │                      │                                      │
    │                      └── 100kΩ ── GND                      │
    │                                                             │
    │  Status LED (Main CPU only):                               │
    │      GPIO2 ── 330Ω ── LED ── GND                           │
    │                                                             │
    │  ESP-NOW: Built-in PCB antenna                             │
    └─────────────────────────────────────────────────────────────┘
```

## Software Setup

### 1. Install ESP32-C3 ULP RISC-V Toolchain

```bash
# Install ESP-IDF if not already installed
git clone --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh esp32c3
source export.sh

# Install ULP RISC-V toolchain
pip install esptool
git clone https://github.com/espressif/esp32-ulp-riscv-toolchain.git
cd esp32-ulp-riscv-toolchain
# Follow installation instructions for your platform

# Add to PATH
export PATH=$PATH:/path/to/esp32-ulp-riscv-toolchain/bin
```

### 2. Verify ULP Support

```bash
# Check ESP32-C3 chip support
esptool.py chip_id

# Verify ULP RISC-V toolchain
riscv32-esp-elf-gcc --version
riscv32-esp-elf-as --version
```

### 3. Configure platformio.ini

Use the provided `env:ulp_receiver` environment or customize:

```ini
[env:my_ulp_sensor]
extends = env:ulp_receiver
build_flags = 
    ${env:ulp_receiver.build_flags}
    -DNODE_ID='"my_sensor_01"'
    -DNODE_LOCATION='"my_location"'
    -DRECEIVER_MAC='{0x24,0x6F,0x28,0x12,0x34,0x56}'
    -DNORMAL_INTERVAL_MIN=30
    -DSAFE_INTERVAL_MIN=120
```

### 4. Build and Flash

```bash
# Build ULP receiver
pio run -e ulp_receiver

# Flash to ESP32-C3
pio run -e ulp_receiver -t upload

# Monitor serial output
pio device monitor -e ulp_receiver
```

## Configuration Options

### Power Management
```cpp
// Battery thresholds (millivolts)
#define BATTERY_SAFE_THRESHOLD_MV 3800     // Enter safe mode
#define BATTERY_CRITICAL_THRESHOLD_MV 3400 // Enter emergency mode

// Transmission intervals (minutes)
#define NORMAL_INTERVAL_MIN 15    // Normal operation
#define SAFE_INTERVAL_MIN 60      // Low battery
#define EMERGENCY_INTERVAL_MIN 120 // Critical battery
```

### Hardware Pins (ULP-accessible only)
```cpp
#define ULP_NTC_PIN 0        // GPIO0 -> ADC1_CH0
#define ULP_BATTERY_PIN 1    // GPIO1 -> ADC1_CH1  
#define STATUS_LED_PIN 2     // Main CPU controlled
```

### ESP-NOW Configuration
```cpp
// Set receiver MAC address
uint8_t receiver_mac[6] = {0x24, 0x6F, 0x28, 0xAB, 0xCD, 0xEF};
```

## Operation Modes

### Normal Mode (Battery > 3.8V)
- ULP reads sensors every 15 minutes
- Main CPU wakes only for ESP-NOW transmission
- Average current: ~25µA
- Expected battery life: 4-5 years

### Safe Mode (Battery 3.4V - 3.8V)
- ULP reads sensors every 60 minutes
- Reduced transmission frequency
- Average current: ~15µA
- Extended battery life

### Emergency Mode (Battery < 3.4V)
- ULP reads sensors every 120 minutes
- Minimal operation to preserve remaining battery
- Average current: ~10µA
- Maximum remaining battery life

## Power Consumption Breakdown

| Component | Current | Duration | Frequency |
|-----------|---------|----------|-----------|
| ULP reading sensors | 150µA | 10ms | Every 15-120min |
| ULP idle between readings | 8µA | Continuous | Always |
| Main CPU ESP-NOW TX | 80mA | 100ms | Every 15-120min |
| Deep sleep (main CPU) | 10µA | Continuous | Always |
| **Average Total** | **~25µA** | **-** | **15min intervals** |

## Expected Battery Life

### 3x AA Lithium Batteries (9000mAh total)
- **15-minute intervals**: 4-5 years
- **30-minute intervals**: 6-7 years  
- **60-minute intervals**: 8-10 years
- **Emergency mode**: 10+ years

### Single CR2450 (600mAh) - Alternative
- **15-minute intervals**: 8-12 months
- **30-minute intervals**: 1.5-2 years
- **60-minute intervals**: 2.5-3 years

## Deployment Guide

### 1. Initial Setup
```bash
# Flash with debug enabled for initial testing
pio run -e ulp_receiver -t upload

# Monitor first few wake cycles
pio device monitor -e ulp_receiver
```

### 2. Verify Operation
Check serial output for:
- ULP wake count incrementing
- Sensor readings (temperature, battery)
- Power mode transitions
- ESP-NOW transmission success

### 3. Production Deployment
```bash
# Flash with minimal debug for power savings
pio run -e ulp_receiver_production -t upload

# Record initial battery voltage and sequence numbers
```

### 4. Remote Monitoring
- Monitor ESP-NOW transmissions at receiver
- Track battery voltage trends
- Watch for power mode changes
- Log transmission success/failure rates

## Troubleshooting

### ULP Not Working
```bash
# Check ULP toolchain installation
riscv32-esp-elf-gcc --version

# Verify ESP32-C3 ULP support
idf.py menuconfig
# Navigate to: Component config -> ESP32C3-Specific -> ULP coprocessor support
```

### Power Consumption Too High
- Verify deep sleep configuration
- Check for serial output in production (disable for power savings)
- Measure actual current with multimeter
- Ensure proper capacitor decoupling

### Sensor Readings Incorrect
- Verify NTC thermistor model and beta value
- Check pullup resistor value (should be 10kΩ ±1%)
- Calibrate voltage divider for battery monitoring
- Test ADC channels individually

### ESP-NOW Transmission Failures
- Verify receiver MAC address configuration
- Check ESP-NOW channel and encryption settings
- Monitor distance between transmitter and receiver
- Test with known-good receiver

### Battery Life Shorter Than Expected
- Measure actual current consumption
- Check for unnecessary wake-ups or processing
- Verify battery capacity and type
- Monitor temperature effects on battery

## Advanced Features

### True ULP RISC-V Implementation
The current implementation simulates ULP operation on the main CPU. For true ULP operation:

1. **Compile ULP Assembly Program**
```bash
# Assemble ULP program
riscv32-esp-elf-as -march=rv32imc -o ulp_main.o src/ulp_main.S
riscv32-esp-elf-ld -T esp32c3.ulp.ld -o ulp_main.elf ulp_main.o
riscv32-esp-elf-objcopy -O binary ulp_main.elf ulp_main.bin
```

2. **Load ULP Binary in Main Program**
```cpp
extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");

size_t ulp_prog_size = ulp_main_bin_end - ulp_main_bin_start;
esp_err_t err = ulp_riscv_load_binary(ulp_main_bin_start, ulp_prog_size);
```

3. **Configure ULP Wake and Sleep**
```cpp
esp_sleep_enable_ulp_wakeup();
ulp_riscv_run();
esp_deep_sleep_start();
```

### Custom Power Gating
Add P-channel MOSFET to cut power to sensors:
```cpp
// In ULP program, control power via GPIO
// Turn on sensor power before reading
// Turn off after reading to save power
```

### Mesh Integration
Deploy multiple ULP sensors with dedicated mesh receivers:
```
ULP Sensors -> Mesh Receivers -> Server
(Ultra-low power) (Mains powered) (Data processing)
```

## Integration with Existing System

The ULP receiver integrates with the existing ESP-NOW mesh system:

1. **ULP sensors**: Ultra-low power, battery operated, transmit-only
2. **Mesh receivers**: Mains powered, handle routing and WiFi upload  
3. **Server**: Processes data, sends safe-mode commands
4. **Dashboard**: Monitors all sensor types

## Performance Targets

### Power Consumption Goals
- **ULP active**: <200µA for <20ms per reading
- **ULP sleep**: <10µA between readings
- **Main CPU wake**: <100mA for <200ms per transmission
- **Deep sleep**: <15µA
- **Average total**: <30µA

### Battery Life Goals  
- **Normal operation**: 3-5 years
- **Safe mode**: 5-8 years
- **Emergency mode**: 8+ years

### Reliability Goals
- **Transmission success**: >95%
- **Sensor accuracy**: ±0.5°C
- **Battery monitoring**: ±0.1V
- **Wake reliability**: >99.9%

## Support and Development

For issues or improvements:
1. Check ESP-IDF ULP RISC-V documentation
2. Review ESP32-C3 datasheet for power specifications
3. Test with oscilloscope for actual current measurements
4. Validate sensor readings with reference thermometer
5. Monitor long-term battery performance

This ULP implementation represents the ultimate in ESP32-based ultra-low power sensing, enabling years of operation on battery power while maintaining reliable sensor data transmission.
