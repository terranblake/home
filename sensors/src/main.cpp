/********************************************************************
 * ESP32 Temperature Sensor with Thermistor
 *  - NTCLE100E3103JB0 (10 kΩ @25 °C, β25/85 ≈ 3435 K)
 *  - Fixed resistor: 10 kΩ 1 % to 3 V3 rail
 *  - Divider midpoint → A0
 *  - MCP1700-33 powers board from single Li-ion cell
 ********************************************************************/

#include <WiFi.h>
#include <HTTPClient.h>
#include <esp_adc_cal.h>

/************* user settings *****************/
#ifndef WIFI_SSID
const char* WIFI_SSID     = "FromTheLandOfKansas";
#endif

#ifndef WIFI_PASSWORD
const char* WIFI_PASSWORD = "Kansas_6614!";
#endif

#ifndef SERVER_ENDPOINT
const char* ENDPOINT      = "http://192.168.8.123:8080/ingest"; // Your computer's IP
#else
const char* ENDPOINT      = SERVER_ENDPOINT;
#endif

#ifndef NODE_LOCATION
const char* NODE_LOCATION = "living room";
#endif

const char* SENSOR_NAME   = "NTCLE100E3103JB0";

#ifndef SENSOR_PIN
// GPIO36 (VP) appears to be connected to the thermistor based on readings
const int SENSOR_PIN    = 36; // Using GPIO36 (ADC1_CHANNEL_0)
#endif

// Deep-sleep interval (µs). 5 min = 5 * 60 * 1e6
const uint64_t SLEEP_US = 5ULL * 60ULL * 1000000ULL;  // 5 minutes for battery conservation
/*********************************************/

// Thermistor constants for NTCLE100E3103JB0
#ifndef R_FIXED
constexpr float R_FIXED = 10000.0;  // 10 kΩ fixed resistor
#endif

#ifndef BETA
constexpr float BETA    = 3977.0;   // NTCLE100E3103JB0 β25/85 = 3977K
#endif

#ifndef T0_TEMP
constexpr float T0_TEMP = 298.15;   // 25 °C in kelvin
#endif

#ifndef R0
constexpr float R0      = 10000.0;  // 10 kΩ @25 °C
#endif

float readTempC(int& raw, float& v, float& r)
{
  raw = analogRead(SENSOR_PIN);
  v   = raw * (3.3f / 4095.0f);
  r   = (v * R_FIXED) / (3.3f - v);

  float tK = 1.0f / ((1.0f / T0_TEMP) + (1.0f / BETA) * log(r / R0));
  return tK - 273.15f;
}

void setup()
{
  Serial.begin(115200);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) delay(100);
}

void loop()
{
  int raw; float v, r;
  float tC = readTempC(raw, v, r);
  float tF = tC * 9.0f / 5.0f + 32.0f;

  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    String url = String(ENDPOINT) +
                 "?room="       + NODE_LOCATION +
                 "&sensor="     + SENSOR_NAME +
                 "&tempC="      + String(tC, 2) +
                 "&tempF="      + String(tF, 2) +
                 "&rawADC="     + String(raw) +
                 "&voltage="    + String(v, 3) +
                 "&resistance=" + String(r, 1) +
                 "&beta="       + String(BETA, 0) +
                 "&r0="         + String(R0, 0) +
                 "&rFixed="     + String(R_FIXED, 0);

    http.begin(url);
    http.GET();
    http.end();
  }
  delay(1000);
}
