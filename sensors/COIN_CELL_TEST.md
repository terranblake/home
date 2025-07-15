# ESP32-C3 SuperMini Coin Cell Battery Test (Simplified)

This simplified test validates that an ESP32-C3 SuperMini can operate on a 3.0V coin cell battery while reading from its ADC and publishing results using ESP-NOW.

## Hardware Setup

1. Connect the ESP32-C3 SuperMini to a 3.0V coin cell battery (CR2032 or similar)
2. Connect components as shown in the circuit diagram below:

```
┌─────────────────────────────────────────────────────────────┐
│  CR2032 Battery (3.0V, ~220-240mAh)                        │
│      (+) ──┬── 100µF ceramic ──┬── ESP32-C3 VCC           │
│            │                   │                           │
│            └── 47µF electro ────┘                          │
│      (-) ────────────────────────── ESP32-C3 GND          │
│                                                             │
│  Thermistor Circuit:                                        │
│      3.3V ── 10kΩ ──┬── GPIO0                             │
│                     │                                       │
│                     └── Thermistor (10kΩ) ── GND          │
│                                                             │
│  Battery Monitoring Circuit:                               │
│      3.3V ── 100kΩ ──┬── GPIO1                            │
│                      │                                      │
│                      └── 100kΩ ── GND                     │
│                                                             │
│  Status LED:                                                │
│      GPIO2 ── 330Ω ── LED ── GND                           │
└─────────────────────────────────────────────────────────────┘
```

## Test Components

1. **Deep Sleep**: The ESP32-C3 uses deep sleep to conserve power, waking periodically
2. **ADC Reading**: Measures temperature from a thermistor and monitors battery voltage
3. **ESP-NOW**: Transmits data via ESP-NOW before returning to deep sleep
4. **Power Efficiency**: Optimized for low power consumption

## Running the Test

1. Connect your ESP32-C3 SuperMini to your computer
2. Verify the device is connected at `/dev/cu.usbmodem3101`
3. Build and upload the test:

```bash
cd sensors
pio run -e coin_cell_test -t upload
```

4. Monitor the output:

```bash
pio device monitor -p /dev/cu.usbmodem3101 -b 115200
```

5. After verifying functionality with USB power, disconnect USB and connect to the coin cell battery

## Expected Results

When functioning correctly, the test will:

1. Wake up periodically (default: every 5 minutes)
2. Take ADC readings for temperature and battery voltage
3. Display status via LED patterns (see below)
4. Transmit data via ESP-NOW
5. Return to deep sleep

### LED Status Patterns

The status LED will show the following patterns to indicate operation:

1. **ADC Reading Success**: One long blink (200ms)
2. **Battery Level**: N short blinks where N corresponds to battery level
   - 1 blink: <500mV
   - 2 blinks: 500-1000mV
   - 3 blinks: 1000-1500mV
   - 4 blinks: 1500-2000mV
   - 5 blinks: >2000mV
3. **ESP-NOW Transmission**:
   - Success: One solid light for 1 second
   - Failure: Five rapid blinks
4. **Going to Sleep**: Three slow blinks

## Optimizing Power Consumption

For maximum battery life with a coin cell:

1. Increase sleep interval (modify the `SLEEP_TIME_US` constant in the code)
2. Remove or disable the status LED in production
3. Use ceramic capacitors for cleaner power with less leakage
4. Ensure all unused pins are set to INPUT_PULLUP or INPUT_PULLDOWN
5. Consider further lowering the CPU frequency in the platformio.ini file

## Customizing the Test

To modify the receiver MAC address, update the `receiverMac` array in `coin_cell_test.cpp`.

To change the wake-up interval, modify the `SLEEP_TIME_US` constant (in microseconds).
