/********************************************************************
 * ESP32 Temperature Sensor with Thermistor
 *  - NTCLE100E3103JB0 (10 kΩ @25 °C, β25/85 ≈ 3435 K)
 *  - Fixed resistor: 10 kΩ 1 % to 3 V3 rail
 *  - Divider midpoint → A0
 *  - MCP1700-33 powers board from single Li-ion cell
 ********************************************************************/

#include <WiFi.h>
#include <HTTPClient.h>

/************* user settings *****************/
const char* WIFI_SSID     = "FromTheLandOfKansas";
const char* WIFI_PASSWORD = "Kansas_6614!";
const char* ENDPOINT      = "http://192.168.8.123:8080/ingest"; // Your computer's IP
const char* ROOM_NAME     = "hallway-apex";
const char* SENSOR_NAME   = "NTCLE100E3103JB0";
const int SENSOR_PIN    = 0; // GPIO0 is ADC1_CH0 on ESP32-C3

// Deep-sleep interval (µs). 5 min = 5 * 60 * 1e6
const uint64_t SLEEP_US = 5ULL * 60ULL * 1000000ULL;  // 5 minutes for battery conservation
/*********************************************/

// Thermistor constants for NTCLE100E3103JB0
constexpr float R_FIXED = 10000.0;  // 10 kΩ fixed resistor
constexpr float BETA    = 3977.0;   // NTCLE100E3103JB0 β25/85 = 3977K
constexpr float T0_TEMP = 298.15;   // 25 °C in kelvin
constexpr float R0      = 10000.0;  // 10 kΩ @25 °C

float readThermistorC()
{
  int raw = analogRead(SENSOR_PIN);               // GPIO0 is ADC1_CH0 on ESP32-C3
  Serial.printf("Raw ADC: %d\n", raw);
  // ESP32-C3 voltage reference (different from ESP32)
  float v  = raw * (3.3 / 4095.0);       // volts at GPIO0 (ESP32-C3 is 12-bit)
  Serial.printf("Voltage: %.3f V\n", v);
  
  if (v >= 3.3) {
    Serial.println("Error: Voltage at or above supply - check thermistor connection");
    return -999.0;
  }
  
  float rT = (v * R_FIXED) / (3.3 - v);   // voltage-divider math
  Serial.printf("Thermistor resistance: %.1f Ω\n", rT);
  
  float tempK = 1.0 / ((1.0 / T0_TEMP) + (1.0 / BETA) * log(rT / R0));
  return tempK - 273.15;                  // °C
}

void setup()
{
  // ESP32-C3 Low Power Configuration
  setCpuFrequencyMhz(80);  // Reduce CPU frequency for power savings
  
  Serial.begin(115200);
  delay(100);
  
  Serial.println("ESP32-C3 Low Power Temperature Sensor");
  Serial.printf("CPU Frequency: %d MHz\n", getCpuFrequencyMhz());
  
  // Configure WiFi with power management
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);  // Disable WiFi sleep during connection
  
  Serial.printf("Connecting to WiFi: %s\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  uint32_t tStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - tStart < 15000) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("WiFi connected! IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.printf("WiFi connection failed. Status: %d\n", WiFi.status());
  }

  float tC = readThermistorC();
  float tF = tC * 9.0 / 5.0 + 32.0;

  Serial.printf("Temp: %.2f °C  (%.2f °F)\n", tC, tF);

  if (WiFi.status() == WL_CONNECTED) {
    WiFiClient client;
    HTTPClient http;
    
    // Get all sensor values for transmission
    int raw = analogRead(SENSOR_PIN);
    float v = raw * (3.3 / 4095.0);
    float rT = (v * R_FIXED) / (3.3 - v);
    
    // Build URL with all parameters
    String url = String(ENDPOINT) + 
                "?room=" + String(ROOM_NAME) +
                "&sensor=" + String(SENSOR_NAME) +
                "&tempC=" + String(tC, 2) +
                "&tempF=" + String(tF, 2) +
                "&rawADC=" + String(raw) +
                "&voltage=" + String(v, 3) +
                "&resistance=" + String(rT, 1) +
                "&beta=" + String(BETA, 0) +
                "&r0=" + String(R0, 0) +
                "&rFixed=" + String(R_FIXED, 0);
    
    Serial.printf("Sending: %s\n", url.c_str());
    http.begin(client, url);
    int httpCode = http.GET();
    Serial.printf("HTTP response: %d\n", httpCode);
    http.end();
    Serial.println("Data sent successfully!");
  } else {
    Serial.println("WiFi not connected - skipping data transmission");
  }

  // Disconnect WiFi to save power
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  
  // RECOVERY MODE - Stay awake for debugging
  Serial.println("RECOVERY MODE: Staying awake, no deep sleep");
  Serial.println("Board is responsive for programming");
  
  // Comment out deep sleep for recovery
  // esp_sleep_enable_timer_wakeup(SLEEP_US);
  // esp_deep_sleep_start();
}

void loop() { 
  // Recovery mode - just blink and stay alive
  Serial.println("Recovery mode - board staying responsive");
  delay(5000);  // 5 second delay instead of deep sleep
}
