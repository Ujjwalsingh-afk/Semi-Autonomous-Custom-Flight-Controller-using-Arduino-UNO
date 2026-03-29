#include <Wire.h>
#include <MPU6050.h>

MPU6050 imu;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  imu.initialize();

  if (imu.testConnection()) {
    Serial.println("MPU-6050 OK");
  } else {
    Serial.println("MPU-6050 connection failed!");
    while(true);
  }
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz;
  imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Convert to angles
  float roll  = atan2(ay, az) * 180.0 / PI;
  float pitch = atan2(-ax, az) * 180.0 / PI;

  Serial.print("Roll: ");
  Serial.print(roll, 1);
  Serial.print("  Pitch: ");
  Serial.print(pitch, 1);
  Serial.print("  GyroX: ");
  Serial.print(gx / 131.0, 1);
  Serial.print("  GyroY: ");
  Serial.println(gy / 131.0, 1);

  delay(200);
}
