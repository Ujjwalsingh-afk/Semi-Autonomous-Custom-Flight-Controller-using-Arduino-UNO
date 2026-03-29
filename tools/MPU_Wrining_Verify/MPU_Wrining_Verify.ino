#include <Wire.h>

void setup() {
  Serial.begin(115200);
  Wire.begin();

  Serial.println("Scanning I2C bus...");

  Wire.beginTransmission(0x68);
  int error = Wire.endTransmission();

  if (error == 0) {
    Serial.println("SUCCESS — MPU-6050 found at address 0x68");
    Serial.println("Wiring is correct. Proceed to Step 2.");
  } else {
    Serial.println("FAILED — MPU-6050 not found!");
    Serial.println("Check these wires:");
    Serial.println("  VCC → Arduino 3.3V (NOT 5V)");
    Serial.println("  GND → Arduino GND");
    Serial.println("  SCL → Arduino A5");
    Serial.println("  SDA → Arduino A4");
    Serial.println("  AD0 → Arduino GND");
  }
}

void loop() {}
