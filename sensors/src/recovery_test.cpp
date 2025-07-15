#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <driver/adc.h>
#include "esp_log.h"

// ESP logging tag
static const char *TAG = "RECOVERY";

// Configuration
#define STATUS_LED_PIN   2      // LED to indicate activity

void setup() {
  // Wait a moment before initializing
  delay(1000);
  
  // Initialize serial
  Serial.begin(115200);
  
  // Initialize status LED
  pinMode(STATUS_LED_PIN, OUTPUT);
  
  ESP_LOGI(TAG, "ESP32-C3 Recovery Mode");
  ESP_LOGI(TAG, "=====================");
  ESP_LOGI(TAG, "This is a recovery firmware that doesn't use deep sleep.");
  ESP_LOGI(TAG, "LED will blink continuously to indicate the device is working.");
}

void loop() {
  // Simple blinking pattern to show the device is working
  digitalWrite(STATUS_LED_PIN, HIGH);
  delay(100);
  digitalWrite(STATUS_LED_PIN, LOW);
  delay(900);
  
  // Print a status message with different baud rates
  Serial.begin(9600);
  ESP_LOGI(TAG, "ESP32-C3 is running in recovery mode (9600)");
  Serial.flush();
  
  digitalWrite(STATUS_LED_PIN, HIGH);
  delay(100);
  digitalWrite(STATUS_LED_PIN, LOW);
  delay(900);
  
  Serial.begin(115200);
  ESP_LOGI(TAG, "ESP32-C3 is running in recovery mode (115200)");
  Serial.flush();
}
