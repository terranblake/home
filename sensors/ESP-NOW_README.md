# ESP-NOW Ultra-Low Power Temperature Sensors

This project implements an ultra-low power temperature monitoring system using ESP-NOW protocol with CR2450 batteries.

## 📋 System Overview

### Architecture
```
[Battery Sensors] --ESP-NOW--> [Receiver] --WiFi--> [Python Server] ---> [Dashboard]
```

- **Battery Transmitters**: ESP32-C3 + CR2450 + NTC thermistor
- **Receiver**: ESP32 (mains powered) forwarding data via WiFi
- **Server**: Existing Python temperature server
- **Dashboard**: Existing Flask web dashboard

## 🔋 Battery Life Estimates

| Interval | Battery Life | Cost/Month* | Use Case |
|----------|-------------|-------------|----------|
| 1 minute | 2.8 days | $32 | Detailed monitoring |
| 5 minutes | 14 days | $6.50 | Good detail |
| 15 minutes | 42 days | $2.15 | Balanced ⭐ |
| 30 minutes | 84 days | $1.08 | General monitoring |
| 1 hour | 168 days | $0.54 | Long-term tracking |

*Assuming CR2450 costs $3 each

## 🛠 Hardware Requirements

### Per Battery Transmitter Node:
- ESP32-C3 development module
- CR2450 lithium battery (540-600mAh, 3V)
- CR2450 battery holder
- NTCLE100E3103JB0 thermistor (10kΩ @25°C)
- 10kΩ 1% precision resistor
- 100µF ceramic capacitor (6.3V, X7R)
- 220µF electrolytic capacitor (6.3V, low ESR)
- Optional: BSS138 P-FET for power gating

### Receiver Node:
- ESP32 or ESP32-C3 development board
- USB power or 5V adapter (mains powered)

## 🔌 Circuit Diagrams

### Battery Transmitter Circuit:
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

### Receiver Circuit:
```
Standard ESP32 development board with USB power
No additional components required
```

## 📁 File Structure

```
src/
├── main.cpp                  # Original WiFi version
├── battery_transmitter.cpp   # Ultra-low power ESP-NOW transmitter
├── espnow_receiver.cpp      # Mains-powered ESP-NOW receiver  
└── recovery_transmitter.cpp  # Debug version (no deep sleep)
```

## 🚀 Setup Instructions

### 1. Flash the Receiver
```bash
# Flash receiver to mains-powered ESP32
pio run -e espnow_receiver -t upload
pio device monitor -e espnow_receiver
```

**Important**: Note the MAC address from Serial output!

### 2. Configure Transmitter
Edit `battery_transmitter.cpp`:
```cpp
// Update with receiver's MAC address
uint8_t receiverMAC[] = {0x30, 0xAE, 0xA4, 0x07, 0x0D, 0x64}; // Your receiver MAC

// Configure node settings
const char* NODE_ID = "bedroom_01";        // Unique ID
const int SLEEP_MINUTES = 15;              // Adjust for battery life
```

### 3. Test with Recovery Version
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
