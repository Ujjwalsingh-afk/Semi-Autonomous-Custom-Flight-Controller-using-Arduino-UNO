#include <Servo.h>

Servo esc1, esc2, esc3, esc4;

void setup() {
  Serial.begin(115200);

  esc1.attach(3);
  esc2.attach(5);
  esc3.attach(6);
  esc4.attach(9);

  // Arm all ESCs first
  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  esc3.writeMicroseconds(1000);
  esc4.writeMicroseconds(1000);

  delay(3000);  // Wait for arming beeps

  Serial.println("==============================");
  Serial.println("  MOTOR DIRECTION TEST");
  Serial.println("==============================");
  Serial.println("Commands:");
  Serial.println("  1 → Spin Motor 1 (Front-Left)  should be CW");
  Serial.println("  2 → Spin Motor 2 (Front-Right) should be CCW");
  Serial.println("  3 → Spin Motor 3 (Rear-Right)  should be CW"); 
  Serial.println("  4 → Spin Motor 4 (Rear-Left)   should be CCW");

  Serial.println("  S → Stop all motors");
  Serial.println("");
  Serial.println("Looking from ABOVE the drone:");
  Serial.println("  CCW = spins anti-clockwise");
  Serial.println("  CW  = spins clockwise");
}

void loop() {
  if (!Serial.available()) return;

  char cmd = Serial.read();

  // Stop all first
  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  esc3.writeMicroseconds(1000);
  esc4.writeMicroseconds(1000);
  delay(500);

  switch(cmd) {
    case '1':
      Serial.println("Spinning Motor 1 (Front-Left) — should be CW");
      esc1.writeMicroseconds(1100);
      delay(3000);
      esc1.writeMicroseconds(1000);
      Serial.println("Stopped. Was it CW? (look from above)");
      break;

    case '2':
      Serial.println("Spinning Motor 2 (Front-Right) — should be CCW");
      esc2.writeMicroseconds(1100);
      delay(3000);
      esc2.writeMicroseconds(1000);
      Serial.println("Stopped. Was it CCW? (look from above)");
      break;

    case '3':
      Serial.println("Spinning Motor 3 (Rear-Right) — should be CW");
      esc3.writeMicroseconds(1100);
      delay(3000);
      esc3.writeMicroseconds(1000);
      Serial.println("Stopped. Was it CW? (look from above)");
      break;

    case '4':
      Serial.println("Spinning Motor 4 (Rear-Left) — should be CCW");
      esc4.writeMicroseconds(1100);
      delay(3000);
      esc4.writeMicroseconds(1000);
      Serial.println("Stopped. Was it CCW? (look from above)");
      break;

    case 'S': case 's':
      Serial.println("All motors stopped.");
      break;
  }
}
