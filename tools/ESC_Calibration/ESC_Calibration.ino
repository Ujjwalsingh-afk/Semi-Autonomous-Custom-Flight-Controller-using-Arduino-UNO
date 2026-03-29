#include <Servo.h>
       
Servo esc1, esc2, esc3, esc4;

void setup() {
  Serial.begin(115200);

  esc1.attach(3);
  esc2.attach(5);
  esc3.attach(6);
  esc4.attach(9);

  // Write MAXIMUM immediately on startup
  esc1.writeMicroseconds(2000);
  esc2.writeMicroseconds(2000);
  esc3.writeMicroseconds(2000);
  esc4.writeMicroseconds(2000);

  Serial.println("=====================================");
  Serial.println("       ESC CALIBRATION READY");
  Serial.println("=====================================");
  Serial.println("");
  Serial.println("All ESCs receiving MAXIMUM signal.");
  Serial.println("");
  Serial.println(">>> CONNECT LIPO BATTERY NOW <<<");
  Serial.println("");
  Serial.println("Wait for beeps from ALL 4 ESCs...");
  Serial.println("When ALL 4 have beeped → type Y + Enter");
}

bool step1done = false;

void loop() {
  if (Serial.available()) {
    char c = Serial.read();

    if ((c == 'Y' || c == 'y') && !step1done) {
      step1done = true;

      Serial.println("");
      Serial.println("Writing MINIMUM throttle now...");
      Serial.println("Wait for confirmation beeps...");

      esc1.writeMicroseconds(1000);
      esc2.writeMicroseconds(1000);
      esc3.writeMicroseconds(1000);
      esc4.writeMicroseconds(1000);

      delay(4000);

      Serial.println("");
      Serial.println("=====================================");
      Serial.println("     CALIBRATION COMPLETE");
      Serial.println("=====================================");
      Serial.println("All 4 ESCs calibrated successfully.");
      Serial.println("");
      Serial.println("DISCONNECT LIPO now.");
      Serial.println("Then upload main flight code.");
    }
  }
}