# ESP32 Mesh Temperature Sensor Network with Smart Battery Protection

## Overview

This system implements an ultra-low-power ESP32-C3 mesh sensor network using ESP-NOW with intelligent battery protection. The server automatically detects temperature anomalies (high standard deviation, rapid increases) that correlate with battery drain and automatically triggers safe-mode to preserve battery life.

## Network Architecture

```
[Battery Transmitters] ←--ESP-NOW--→ [Battery/Mains Receivers] ←--WiFi--→ [Python Server]
        │                                    │                           │
        └── Ultra-low power                  └── Mesh relay               └── Anomaly detection
        └── Safe-mode capable                └── Safe-mode control        └── Safe-mode commands
```

## Hardware Configurations

### Battery Transmitter (Ultra-Low Power)
- **Power**: CR2450 3V battery (540-600mAh)
- **MCU**: ESP32-C3 module
- **Sensor**: NTCLE100E3103JB0 NTC thermistor
- **Expected Life**: 15min intervals = ~42 days, 1hr intervals = ~168 days
- **Safe Mode Life**: 300+ days with 1hr intervals

### Receiver Options

#### Option A: Mains-Powered Gateway
- **Power**: USB/5V mains supply
- **MCU**: ESP32 dev board
- **Function**: Primary gateway, always-on WiFi, HTTP forwarding

#### Option B: Battery-Powered Mesh Node  
- **Power**: 3x AA Lithium batteries (4.5V, ~9000mAh total)
- **MCU**: ESP32-C3 module
- **Expected Life**: Normal mode ~30-45 days, Safe mode ~90-120 days
- **Function**: Mesh relay, range extension, smart power management

## Circuit Diagrams

### Battery Transmitter Circuit
```
CR2450 Battery (3.0V, 540-600mAh)
    (+) ──┬── 100µF ceramic ──┬── ESP32-C3 VCC
          │                   │
          └── 220µF electro ──┘
    (-) ────────────────────────── ESP32-C3 GND

Thermistor Circuit:
    3.3V ── 10kΩ ── GPIO4 ── NTC(10kΩ) ── GND

Optional Battery Monitor:
    3.3V ── 100kΩ ── GPIO3 ── 100kΩ ── GND
```

### Battery-Powered Mesh Receiver Circuit
```
3x AA Lithium Batteries (4.5V, ~3000mAh each)
    (+) ──┬── 470µF electro ──┬── ESP32-C3 VCC
          │                   │
          └── 100µF ceramic ──┘
    (-) ────────────────────────── ESP32-C3 GND

Battery Monitor:
    3.3V ── 100kΩ ── GPIO3 ── 100kΩ ── GND

Status LED:
    GPIO2 ── 330Ω ── LED ── GND
```

## Smart Battery Protection System

### Anomaly Detection (Server-Side)

The Python server continuously monitors temperature data for patterns that indicate battery degradation:

1. **High Standard Deviation**: σ > 2.0°C indicates erratic readings
2. **Rapid Temperature Rise**: >5.0°C increase in 5 minutes  
3. **Statistical Outliers**: Z-score > 4 from recent averages

### Safe Mode Triggers

Safe mode is automatically triggered when:
- Battery voltage drops below thresholds
- Multiple consecutive transmission failures
- Server detects temperature anomalies
- Manual command from server/receiver

### Power Modes

| Mode | Transmitter Interval | Receiver Check Interval | Battery Life |
|------|---------------------|------------------------|--------------|
| **Normal** | 15 minutes | 5 seconds | Baseline |
| **Safe** | 60 minutes | 30 seconds | 5-10x longer |
| **Emergency** | 4 hours | 2 minutes | 20-30x longer |

## ESP-NOW Mesh Features

### Data Relay
- Automatic packet forwarding between nodes
- Range extension beyond direct WiFi coverage
- Load balancing across multiple paths
- Loop prevention and intelligent routing

### Safe Mode Command Propagation
- Server sends commands to receivers via HTTP
- Receivers broadcast commands via ESP-NOW  
- Transmitters relay commands to other transmitters
- Authenticated commands with cooldown periods

### Mesh Topology Example
```
[Transmitter A] ──┐
                  ├── [Mesh Receiver] ──WiFi── [Server]
[Transmitter B] ──┘         │
                            └── [Transmitter C] (relay node)
                                     │
                                [Transmitter D] (extended range)
```

## Configuration Management

### PlatformIO Configuration-Driven System

All configuration is now externalized to `platformio.ini` with no need to modify source code for deployment:

```ini
; Default configuration inherited by all environments
[env]
build_flags = 
    ; Network Configuration
    -DWIFI_SSID=\"YourNetworkName\"
    -DWIFI_PASSWORD=\"YourPassword\"
    -DSERVER_ENDPOINT=\"http://192.168.1.100:8080/ingest\"
    
    ; Node Configuration  
    -DNODE_ID=\"sensor_01\"
    -DNODE_LOCATION=\"living_room\"
    -DNODE_TYPE=\"temperature\"
    
    ; Hardware Configuration
    -DSENSOR_PIN=4
    -DBATTERY_PIN=3
    -DSTATUS_LED_PIN=2
    
    ; Timing (minutes)
    -DNORMAL_SLEEP_MINUTES=15
    -DSAFE_SLEEP_MINUTES=60
    -DEMERGENCY_SLEEP_MINUTES=240
    
    ; Power Management
    -DBATTERY_SAFE_THRESHOLD=2.4
    -DBATTERY_NORMAL_THRESHOLD=2.6
    -DBATTERY_CRITICAL_THRESHOLD=2.2
    
    ; Feature Flags
    -DBATTERY_POWERED=false
    -DMESH_RELAY_ENABLED=true
    -DSAFE_MODE_CONTROL_ENABLED=true
```

### Example Deployments

**Bedroom Sensor:**
```ini
[env:bedroom_sensor]
extends = env:battery_transmitter
build_flags = 
    ${env:battery_transmitter.build_flags}
    -DNODE_ID=\"bedroom_01\"
    -DNODE_LOCATION=\"master_bedroom\"
    -DNORMAL_SLEEP_MINUTES=30
    -DRECEIVER_MAC_0=0x30  ; Your receiver's MAC
    -DRECEIVER_MAC_1=0xAE
    ; ... (full MAC address)
```

**Living Room Hub:**
```ini
[env:living_room_hub]
extends = env:mesh_node
build_flags = 
    ${env:mesh_node.build_flags}
    -DNODE_ID=\"living_room_hub\"
    -DWIFI_SSID=\"GuestNetwork\"
    -DSERVER_ENDPOINT=\"http://192.168.1.200:8080/ingest\"
```

**Outdoor Sensor:**
```ini
[env:outdoor_sensor]
extends = env:battery_transmitter
build_flags = 
    ${env:battery_transmitter.build_flags}
    -DNODE_ID=\"outdoor_01\"
    -DNORMAL_SLEEP_MINUTES=60
    -DBATTERY_SAFE_THRESHOLD=2.3  ; Conservative for cold
    -DUSE_BATTERY_VOLTAGE_DIVIDER=true
```

## Deployment Guide

### Quick Start (Zero Code Changes Required)

**1. Build and Deploy Transmitter:**
```bash
cd sensors
# Deploy bedroom sensor with custom settings
pio run -e bedroom_sensor -t upload
# Monitor to get MAC address
pio device monitor -e bedroom_sensor
```

**2. Build and Deploy Receiver/Gateway:**
```bash
# Update receiver MAC in platformio.ini first
pio run -e espnow_receiver -t upload
```

**3. Deploy Additional Mesh Nodes:**
```bash
# Battery-powered mesh nodes for range extension
pio run -e mesh_node -t upload
```

**4. Start Fast-Boot Server:**
```bash
cd sensors
python3 temp_server.py
# Server starts immediately - no startup delay!
# Historical data loads in background
```

### Production Deployment Benefits

- **🚀 Fast Server Boot**: Data ingestion starts immediately (< 2 seconds)
- **⚙️ Zero Code Changes**: All configuration in `platformio.ini`
- **🔄 Quick Redeployment**: Minimal data loss during server restarts
- **📝 Configuration Management**: Version control friendly settings
- **🎯 Environment-Specific**: Easy multi-location deployments

## Monitoring and Troubleshooting

### Serial Output Monitoring
- **Transmitters**: Show power mode, sleep intervals, relay activity
- **Receivers**: Show packet counts, relay statistics, safe mode commands
- **Server**: Shows anomaly detection, statistical analysis, safe mode triggers

### LED Status Indicators
- **Normal**: Slow blink (2s intervals)
- **Safe Mode**: Medium blink (5s intervals)  
- **Emergency**: Slow blink (10s intervals)
- **Activity**: Fast blink during packet transmission/relay

### Expected Battery Life

| Configuration | Normal Mode | Safe Mode | Emergency Mode |
|---------------|-------------|-----------|----------------|
| **CR2450 Transmitter** | 14-168 days | 200-300 days | 400+ days |
| **3x AA Receiver** | 30-45 days | 90-120 days | 180+ days |

## Real-World Performance

Based on the temperature graph showing erratic readings during battery degradation:
- High variance readings correlate with battery drain
- Rapid temperature increases indicate failing battery/connections  
- Safe mode activation preserves remaining battery for months
- Mesh relay ensures data delivery even with reduced transmission frequency

## Future Enhancements

1. **Advanced Mesh Routing**: Implement AODV or similar routing protocol
2. **LoRaWAN Integration**: For even longer range with ultra-low power
3. **Machine Learning**: Predictive battery failure detection
4. **Solar Charging**: For permanent outdoor deployment
5. **Encrypted Communication**: Enhanced security for safe mode commands
6. **Dynamic Network Topology**: Self-healing mesh with automatic route optimization
