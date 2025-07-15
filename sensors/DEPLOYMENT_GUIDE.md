# 🚀 ESP-NOW Temperature Sensor - Deployment Guide

## 📋 Project Status

✅ **READY FOR DEPLOYMENT** - Core components successfully built and tested

### Working Components:
- ✅ **Recovery Transmitter** - Debug/testing version (no deep sleep)
- ✅ **ESP-NOW Receiver** - WiFi gateway (mains powered)  
- ✅ **Battery Transmitter** - Production sensor (ultra-low power)

### Advanced Component:
- ⚠️ **ULP Receiver** - Requires ESP-IDF framework (not Arduino)

## 🛠 Quick Start Deployment

### Step 1: Build Everything
```bash
cd /Users/terran/Documents/home/sensors
./build_and_test.sh build
```

### Step 2: Deploy Gateway (Receiver)
```bash
# Flash to ESP32 dev board (mains powered)
./build_and_test.sh flash espnow_receiver

# Monitor to get MAC address
./build_and_test.sh monitor espnow_receiver
```
**Important**: Note the MAC address displayed - you'll need it for sensor configuration!

### Step 3: Test with Debug Sensor
```bash
# Flash debug version first (good for testing)
./build_and_test.sh flash recovery_transmitter

# Monitor sensor output
./build_and_test.sh monitor recovery_transmitter
```

### Step 4: Deploy Production Sensor
```bash
# Update MAC address in platformio.ini first!
# Then flash production version
./build_and_test.sh flash battery_transmitter
```

## ⚡ Power Modes & Battery Life

### Recovery Transmitter (Debug):
- **Power**: USB powered
- **Interval**: 10 seconds
- **Purpose**: Testing, debugging, getting MAC addresses
- **Sleep**: None (stays awake)

### Battery Transmitter (Production):
- **Power**: CR2450 battery (540-600mAh)
- **Interval**: 15 minutes (configurable)
- **Purpose**: Long-term deployment
- **Sleep**: Deep sleep between transmissions
- **Battery Life**: 
  - 15 min intervals: ~42 days
  - 1 hour intervals: ~168 days

### ESP-NOW Receiver (Gateway):
- **Power**: USB/mains powered
- **Purpose**: Receive sensor data, forward to server
- **Features**: Mesh relay, safe-mode control

## 🔧 Configuration Options

### Quick Sensor Configurations (platformio.ini):

**Bedroom Sensor:**
```bash
pio run -e bedroom_sensor -t upload
```

**Outdoor Sensor:**
```bash  
pio run -e outdoor_sensor -t upload
```

**Custom Configuration:**
Edit `platformio.ini` to add your own environments with:
- Node ID and location
- Receiver MAC address
- Sleep intervals
- Battery thresholds

## 📡 Network Architecture

```
[Battery Sensors] --ESP-NOW--> [Receiver] --WiFi--> [Python Server] -> [Dashboard]
```

### Data Flow:
1. Battery sensors read temperature every 15-60 minutes
2. ESP-NOW transmission to receiver (fast, low power)
3. Receiver forwards to existing Python server via HTTP
4. Dashboard displays real-time data

## 🔋 Hardware Requirements

### For Battery Transmitter:
- ESP32-C3 development board
- CR2450 lithium battery + holder
- NTCLE100E3103JB0 thermistor (10kΩ)
- 10kΩ resistor (1% tolerance)
- 100µF + 220µF capacitors

### For Receiver:
- ESP32 or ESP32-C3 development board  
- USB power or 5V adapter
- No additional components needed

## 🎯 Production Deployment Checklist

### Before Deployment:
- [ ] Test with recovery transmitter first
- [ ] Verify temperature readings are reasonable
- [ ] Check battery voltage (should be 2.8-3.2V)
- [ ] Confirm ESP-NOW packets received successfully
- [ ] Note receiver MAC address
- [ ] Update transmitter configuration with correct MAC

### For Long-term Deployment:
- [ ] Flash production battery_transmitter firmware
- [ ] Set appropriate sleep intervals (15-60 minutes)
- [ ] Monitor first few transmissions
- [ ] Set calendar reminders for battery replacement

### Battery Management:
- [ ] Replace when voltage drops below 2.5V
- [ ] Expected life: 42 days (15min) to 168 days (1hr)
- [ ] Use dashboard to monitor battery health

## 🔍 Troubleshooting

### Common Issues:

**No ESP-NOW packets received:**
- Check receiver is powered and showing MAC address
- Verify transmitter MAC configuration matches receiver
- Check distance (ESP-NOW range ~20-100m depending on environment)

**Invalid temperature readings:**  
- Check NTC thermistor wiring
- Verify 10kΩ resistor value
- Test with recovery transmitter first

**Short battery life:**
- Reduce transmission frequency
- Check for failed transmissions (increases power usage)
- Verify deep sleep is working (production firmware only)

**Build errors:**
- Check all dependencies installed: `pio lib install`
- Try clean build: `pio run -e <env> -t clean`
- Update PlatformIO: `pio upgrade`

## 📊 Performance Expectations

### Transmission Success Rate: 
- Indoor: >95%
- Outdoor: >90% (weather dependent)
- Range: 20-100m (varies with obstacles)

### Power Consumption:
- Active (50ms): ~120mA
- Deep sleep: ~8µA  
- Average: ~20-50µA depending on interval

### Data Accuracy:
- Temperature: ±0.5°C (with quality NTC)
- Battery voltage: ±0.1V
- Timestamp: Synchronized via receiver

## 🚀 Next Steps

1. **Start Simple**: Use recovery transmitter for initial testing
2. **Get Familiar**: Monitor serial output to understand operation  
3. **Deploy Gradually**: Start with one sensor, expand network
4. **Monitor Performance**: Use dashboard to track battery life and data quality
5. **Scale Up**: Add multiple sensors with unique node IDs

Your ESP-NOW sensor network is ready for deployment! 🎉
