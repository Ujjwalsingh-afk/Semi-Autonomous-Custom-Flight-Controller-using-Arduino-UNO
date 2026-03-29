#include <Servo.h>

Servo esc1, esc2, esc3, esc4;

void setup() {
  Serial.begin(115200);

  esc1.attach(3);
  esc2.attach(5);
  esc3.attach(6);
  esc4.attach(9);

  // Send 1000 to all
  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  esc3.writeMicroseconds(1000);
  esc4.writeMicroseconds(1000);

  Serial.println("ESC FAULT FINDER");
  Serial.println("Connect battery now");
  Serial.println("Count how many ESCs beep");
  Serial.println("Then use commands:");
  Serial.println("1 → Send signal to ESC1 only (Pin 3)");
  Serial.println("2 → Send signal to ESC2 only (Pin 5)");
  Serial.println("3 → Send signal to ESC3 only (Pin 6)");
  Serial.println("4 → Send signal to ESC4 only (Pin 9)");
  Serial.println("A → Send signal to ALL ESCs");
  Serial.println("S → Stop all");
}

void loop() {
  if (!Serial.available()) return;
  char cmd = Serial.read();

  // Reset all to 1000 first
  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  esc3.writeMicroseconds(1000);
  esc4.writeMicroseconds(1000);
  delay(200);

  switch(cmd) {
    case '1':
      Serial.println("Testing ESC1 — Pin 3 — Front-Left");
      Serial.println("Does ESC1 beep or respond?");
      esc1.writeMicroseconds(1550);
      delay(2000);
      esc1.writeMicroseconds(1000);
      break;

    case '2':
      Serial.println("Testing ESC2 — Pin 5 — Front-Right");
      Serial.println("Does ESC2 beep or respond?");
      esc2.writeMicroseconds(1550);
      delay(2000);
      esc2.writeMicroseconds(1000);
      break;

    case '3':
      Serial.println("Testing ESC3 — Pin 6 — Rear-Left");
      Serial.println("Does ESC3 beep or respond?");
      esc3.writeMicroseconds(1550);
      delay(2000);
      esc3.writeMicroseconds(1000);
      break;

    case '4':
      Serial.println("Testing ESC4 — Pin 9 — Rear-Right");
      Serial.println("Does ESC4 beep or respond?");
      esc4.writeMicroseconds(1550);
      delay(5000);
      esc4.writeMicroseconds(1000);
      break;

    case 'A': case 'a':
      Serial.println("Testing ALL ESCs together");
      esc1.writeMicroseconds(1150);
      esc2.writeMicroseconds(1150);
      esc3.writeMicroseconds(1150);
      esc4.writeMicroseconds(1150);
      delay(5000);
      esc1.writeMicroseconds(1000);
      esc2.writeMicroseconds(1000);
      esc3.writeMicroseconds(1000);
      esc4.writeMicroseconds(1000);
      break;

    case 'S': case 's':
      Serial.println("All stopped");
      break;
  }
}