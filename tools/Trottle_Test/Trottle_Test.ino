#include <Servo.h>

Servo esc1, esc2, esc3, esc4;

int throttle = 1000;

void setup() {
  Serial.begin(115200);

  esc1.attach(3);
  esc2.attach(5);
  esc3.attach(6);
  esc4.attach(9);

  // Arm ESCs
  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  esc3.writeMicroseconds(1000);
  esc4.writeMicroseconds(1000);

  Serial.println("==============================");
  Serial.println("   HOVER THROTTLE FINDER");
  Serial.println("==============================");
  Serial.println("Connect battery and wait for");
  Serial.println("arming beeps then silence.");
  Serial.println("");
  Serial.println("Commands:");
  Serial.println("  + → Increase throttle by 10");
  Serial.println("  - → Decrease throttle by 10");
  Serial.println("  F → Fine tune +2");
  Serial.println("  f → Fine tune -2");
  Serial.println("  S → Stop all motors (emergency)");
  Serial.println("  R → Read current throttle value");
  Serial.println("");
  Serial.println("Increase slowly until all 4 motors");
  Serial.println("spin at same speed and sound equal.");
  Serial.println("Note that throttle value.");
}

void writeAll(int val) {
  throttle = constrain(val, 1000, 1550);
  esc1.writeMicroseconds(throttle);
  esc2.writeMicroseconds(throttle);
  esc3.writeMicroseconds(throttle);
  esc4.writeMicroseconds(throttle);
}

void loop() {
  if (!Serial.available()) return;

  char cmd = Serial.read();

  switch(cmd) {
    case '+':
      writeAll(throttle + 10);
      Serial.print("Throttle: ");
      Serial.println(throttle);
      break;

    case '-':
      writeAll(throttle - 10);
      Serial.print("Throttle: ");
      Serial.println(throttle);
      break;

    case 'F':
      writeAll(throttle + 2);
      Serial.print("Throttle: ");
      Serial.println(throttle);
      break;

    case 'f':
      writeAll(throttle - 2);
      Serial.print("Throttle: ");
      Serial.println(throttle);
      break;

    case 'S': case 's':
      writeAll(1000);
      Serial.println("STOPPED — Throttle: 1000");
      break;

    case 'R': case 'r':
      Serial.print("Current throttle: ");
      Serial.println(throttle);
      break;
  }
}