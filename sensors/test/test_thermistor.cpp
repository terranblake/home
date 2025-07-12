#include <unity.h>
#include <math.h>

// Copy the thermistor calculation logic for testing
constexpr float R_FIXED = 10000.0;  // 10 kΩ
constexpr float BETA    = 3435.0;   // datasheet β25/85
constexpr float T0      = 298.15;   // 25 °C in kelvin
constexpr float R0      = 10000.0;  // 10 kΩ @25 °C

float calculateTempFromResistance(float rT) {
    float tempK = 1.0 / ((1.0 / T0) + (1.0 / BETA) * log(rT / R0));
    return tempK - 273.15;  // °C
}

float calculateTempFromVoltage(float v) {
    float rT = (v * R_FIXED) / (3.3 - v);   // voltage-divider math
    return calculateTempFromResistance(rT);
}

void test_thermistor_at_25C() {
    // At 25°C, resistance should be 10kΩ
    float temp = calculateTempFromResistance(10000.0);
    TEST_ASSERT_FLOAT_WITHIN(0.1, 25.0, temp);
}

void test_thermistor_voltage_divider() {
    // At 25°C with 10kΩ thermistor and 10kΩ fixed resistor
    // Voltage should be 3.3V / 2 = 1.65V
    float temp = calculateTempFromVoltage(1.65);
    TEST_ASSERT_FLOAT_WITHIN(0.5, 25.0, temp);
}

void test_thermistor_bounds() {
    // Test reasonable temperature range
    float temp_low = calculateTempFromVoltage(0.5);   // Higher resistance, lower temp
    float temp_high = calculateTempFromVoltage(2.8);  // Lower resistance, higher temp
    
    TEST_ASSERT_TRUE(temp_low < temp_high);
    TEST_ASSERT_TRUE(temp_low > -50.0);  // Reasonable lower bound
    TEST_ASSERT_TRUE(temp_high < 150.0); // Reasonable upper bound
}

void setup() {
    delay(2000); // Wait for serial monitor
    
    UNITY_BEGIN();
    RUN_TEST(test_thermistor_at_25C);
    RUN_TEST(test_thermistor_voltage_divider);
    RUN_TEST(test_thermistor_bounds);
    UNITY_END();
}

void loop() {
    // Empty loop
}
