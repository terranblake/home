# ESP-NOW Ultra-Low Power Temperature Sensors

This project implements an ultra-low power temperature monitoring system using ESP-NOW protocol with multiple power configurations for different deployment scenarios.

## 📋 System Overview

### Architecture
```
[Battery Sensors] --ESP-NOW--> [Receiver] --WiFi--> [Python Server] ---> [Dashboard]
```

- **Battery Transmitters**: ESP32-C3 + CR2450/AA + NTC thermistor
- **ULP Receivers**: ESP32-C3 with ULP RISC-V coprocessor (ultra-low power)
- **Mesh Receivers**: ESP32 (mains powered) forwarding data via WiFi
- **Server**: Python temperature server with anomaly detection
- **Dashboard**: Flask web dashboard with real-time monitoring

## 🔋 Power Configuration Options

### 1. Standard Battery Transmitter (CR2450)
- **Power**: Single CR2450 (540-600mAh)
- **Life**: 14-168 days depending on interval
- **Use**: Short to medium-term deployments
- **File**: `battery_transmitter.cpp`

### 2. ULP RISC-V Ultra-Low Power (3x AA Lithium)
- **Power**: 3x AA Lithium (9000mAh total)
- **Life**: 3-8+ years depending on interval
- **Use**: Long-term remote deployments
- **File**: `ulp_receiver.cpp`
- **Average Current**: ~20-30µA

### 3. Recovery/Debug Mode (USB Powered)
- **Power**: USB/mains powered
- **Life**: Continuous operation
- **Use**: Development and debugging
- **File**: `recovery_transmitter.cpp`

## ⚡ ULP RISC-V Ultra-Low Power Features

The ULP (Ultra-Low Power) variant uses the ESP32-C3's RISC-V coprocessor to achieve extreme power efficiency:

### Power Consumption
- **ULP reading sensors**: ~150µA for 10ms every 15-120 minutes
- **ULP sleep**: ~8µA between readings
- **Main CPU wake**: ~80mA for 100ms during transmission
- **Deep sleep**: ~10µA
- **Average total**: ~25µA (vs 200-500µA for standard operation)

### Expected Battery Life (3x AA Lithium)
| Interval | Battery Life | Use Case |
|----------|-------------|----------|
| 15 minutes | 4-5 years | High detail monitoring |
| 30 minutes | 6-7 years | Balanced monitoring |
| 60 minutes | 8-10 years | Long-term tracking |
| Emergency mode | 10+ years | Minimal operation |

### Key Features
- True ULP RISC-V coprocessor operation (when properly configured)
- Main CPU only wakes for ESP-NOW transmission
- Automatic power mode adjustment based on battery voltage
- Years of operation on single battery set
- Ideal for remote, hard-to-access deployments

## 🔋 Standard Battery Life Estimates (CR2450)

| Interval | Battery Life | Cost/Month* | Use Case |
|----------|-------------|-------------|----------|
| 1 minute | 2.8 days | $32 | Detailed monitoring |
| 5 minutes | 14 days | $6.50 | Good detail |
| 15 minutes | 42 days | $2.15 | Balanced ⭐ |
| 30 minutes | 84 days | $1.08 | General monitoring |
| 1 hour | 168 days | $0.54 | Long-term tracking |

*Assuming CR2450 costs $3 each

## 🛠 Hardware Requirements

### Standard Battery Transmitter (CR2450):
- ESP32-C3 development module
- CR2450 lithium battery (540-600mAh, 3V)
- CR2450 battery holder
- NTCLE100E3103JB0 thermistor (10kΩ @25°C)
- 10kΩ 1% precision resistor
- 100µF ceramic capacitor (6.3V, X7R)
- 220µF electrolytic capacitor (6.3V, low ESR)
- Optional: BSS138 P-FET for power gating

### ULP Ultra-Low Power Node (3x AA):
- ESP32-C3-MINI-1 module (ULP RISC-V support required)
- 3x AA lithium batteries (3000mAh each, 4.5V total)
- Battery holder for 3x AA
- NTCLE100E3103JB0 thermistor (10kΩ @25°C)
- 10kΩ 1% precision resistor
- 2x 100kΩ resistors for battery voltage divider
- 470µF electrolytic capacitor (low ESR)
- 100µF ceramic capacitor (0805 or larger)
- Status LED with 330Ω resistor

### Mesh Receiver Node:
- ESP32 or ESP32-C3 development board
- USB power or 5V adapter (mains powered)
- Optional: External antenna for improved range

## 🔌 Circuit Diagrams

### Standard Battery Transmitter (CR2450):
```
CR2450 (+) ──┬── 100µF ceramic ──┬── ESP32-C3 VCC
             │                   │
             └── 220µF electrolytic ──┘
             
CR2450 (-) ──── ESP32-C3 GND

Thermistor Circuit:
ESP32-C3 3.3V ── 10kΩ resistor ── GPIO4 ── NTC thermistor ── GND

Optional Power Gating:
ESP32-C3 GPIO5 ── BSS138 Gate
BSS138 Source ── 10kΩ resistor ── 3.3V
BSS138 Drain ── GPIO4
```

### ULP Ultra-Low Power (3x AA Lithium):
```
3x AA Lithium (4.5V) ──┬── 470µF electro ──┬── ESP32-C3 VCC (3.3V)
                       │                   │
                       └── 100µF ceramic ──┘
                       
Battery (-) ──── ESP32-C3 GND

ULP Thermistor (ULP-accessible pins only):
ESP32-C3 3.3V ── 10kΩ ──┬── GPIO0 (ULP_GPIO0, ADC1_CH0)
                         │
                         └── NTC thermistor ── GND

ULP Battery Monitor:
ESP32-C3 3.3V ── 100kΩ ──┬── GPIO1 (ULP_GPIO1, ADC1_CH1)
                          │
                          └── 100kΩ ── GND

Status LED:
GPIO2 ── 330Ω ── LED ── GND
```

### Mesh Receiver Circuit:
```
Standard ESP32 development board with USB power
No additional components required
Optional: External antenna for improved range
```

## 📁 File Structure

```
src/
├── main.cpp                  # Original WiFi version
├── battery_transmitter.cpp   # Standard battery ESP-NOW transmitter
├── ulp_receiver.cpp         # ULP RISC-V ultra-low power receiver
├── ulp_main.S              # ULP RISC-V assembly program  
├── espnow_receiver.cpp      # Mains-powered ESP-NOW receiver
└── recovery_transmitter.cpp  # Debug version (no deep sleep)

docs/
├── ESP-NOW_README.md        # This file - main documentation
├── ULP_SETUP_GUIDE.md       # Detailed ULP implementation guide
└── MESH_SYSTEM_OVERVIEW.md  # Mesh networking documentation

CMakeLists.txt               # Build configuration for ULP
platformio.ini               # All build environments and configuration
```

## 🚀 Setup Instructions

### 1. Choose Your Deployment Type

#### Standard Battery Deployment (CR2450)
- Use for: 1-6 month deployments, easy battery replacement
- File: `battery_transmitter.cpp`
- Environment: `env:battery_transmitter`

#### ULP Ultra-Low Power Deployment (3x AA)
- Use for: 3+ year deployments, remote locations
- File: `ulp_receiver.cpp`  
- Environment: `env:ulp_receiver`
- See: `ULP_SETUP_GUIDE.md` for detailed setup

#### Debug/Development
- Use for: Testing and development
- File: `recovery_transmitter.cpp`
- Environment: `env:recovery_transmitter`

### 2. Flash the Mesh Receiver
```bash
# Flash receiver to mains-powered ESP32
pio run -e espnow_receiver -t upload
pio device monitor -e espnow_receiver
```

**Important**: Note the MAC address from Serial output!

### 3. Configure Your Sensor Nodes

#### Method A: Using platformio.ini environments
```bash
# Use pre-configured environments
pio run -e outdoor_sensor -t upload      # Outdoor deployment
pio run -e ulp_receiver_shed -t upload   # Shed ULP deployment
pio run -e ulp_receiver_greenhouse -t upload # Greenhouse ULP
```

#### Method B: Custom configuration in platformio.ini
```ini
[env:my_custom_sensor]
extends = env:battery_transmitter  # or env:ulp_receiver
build_flags = 
    ${env:battery_transmitter.build_flags}
    -DNODE_ID='"my_sensor_01"'
    -DNODE_LOCATION='"my_location"'
    -DRECEIVER_MAC='{0x30,0xAE,0xA4,0x07,0x0D,0x64}'  # Your receiver MAC
    -DNORMAL_SLEEP_MINUTES=30
```

### 4. Test with Recovery Version First
```bash
# Flash debug version first (no deep sleep)
pio run -e recovery_transmitter -t upload
pio device monitor -e recovery_transmitter
```

Verify:
- ✅ Battery voltage 2.5-3.2V
- ✅ Temperature readings reasonable
- ✅ ESP-NOW packets sending successfully

### 4. Flash Production Version
```bash
# Flash ultra-low power version
pio run -e battery_transmitter -t upload
```

**Note**: After flashing, the transmitter will go into deep sleep immediately!

## ⚡ Power Optimization Features

### Battery Transmitter:
- **Deep Sleep**: 8µA current draw
- **Fast ESP-NOW**: 50ms active time vs 2-8s WiFi
- **Power Gating**: Optional NTC power control
- **Low CPU Frequency**: 80MHz for power savings
- **Capacitor Buffer**: Handles transmission current peaks

### Receiver:
- **Always On**: No power constraints
- **Full Performance**: 240MHz CPU for fast processing
- **WiFi Management**: Automatic reconnection

## 📊 Monitoring & Debugging

### Serial Output (Receiver):
```
=== ESP-NOW Packet Received ===
From MAC: 30:AE:A4:07:0D:64
Node ID: bedroom_01
Temperature: 23.45°C (74.21°F)
Battery: 2.987V
Sequence: 123
Raw ADC: 2048
```

### Battery Health Monitoring:
- **>2.8V**: Excellent
- **2.5-2.8V**: Good
- **2.3-2.5V**: Low (warning)
- **<2.3V**: Replace battery

## 🔧 Troubleshooting

### Transmitter Issues:
- **No packets**: Check battery voltage, use recovery version
- **Invalid temperature**: Verify NTC wiring and resistor values
- **Short battery life**: Reduce transmission frequency
- **Not responding**: Press reset during power-up

### Receiver Issues:
- **WiFi fails**: Check credentials and signal strength
- **HTTP errors**: Verify server endpoint and network
- **No packets received**: Check MAC address configuration

### Communication Issues:
- **Packets lost**: Reduce distance, check interference
- **Inconsistent data**: Check power supply stability
- **Sequence gaps**: Normal for deep sleep operation

## 🎯 Production Deployment

### Recommended Settings:
- **Interval**: 15-30 minutes for balanced performance
- **Multiple Nodes**: Deploy 3-4 sensors for room comparison
- **Battery Replacement**: Set calendar reminders based on interval

### Scaling Up:
```cpp
// Add multiple transmitters with unique IDs:
bedroom_01, living_room_01, kitchen_01, outdoor_01
```

### Integration:
- Receiver forwards to existing Python server
- Dashboard automatically shows all nodes
- CSV logging includes node_id for multi-room analysis

## 🔄 Maintenance

### Regular Tasks:
- Monitor battery voltage in dashboard
- Replace batteries when <2.5V
- Check for missed packets (sequence gaps)
- Update firmware for improvements

### Seasonal Adjustments:
- Reduce frequency in stable seasons
- Increase monitoring during weather changes
- Add outdoor sensors for comparison

## 📈 Future Enhancements

### Possible Additions:
- **Local Storage**: Buffer data during receiver downtime
- **Mesh Networking**: Sensors relay through each other
- **Additional Sensors**: Humidity, pressure, light
- **Over-the-Air Updates**: Remote firmware updates
- **Solar Charging**: Small solar panel for indefinite operation

This ESP-NOW implementation provides a robust, ultra-low power solution for distributed temperature monitoring with excellent battery life and easy deployment!
