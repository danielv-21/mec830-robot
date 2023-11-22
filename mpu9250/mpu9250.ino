#include "MPU9250.h"

MPU9250 mpu;

float initAngle;
float yawAngle;
float input;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(2000);

  if (!mpu.setup(0x68)) {  // change to your own address
      while (1) {
          Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
          delay(5000);
      }
  }
}

void loop() {
  if (mpu.update()) {
    static uint32_t prev_ms = millis();
    if (millis() > prev_ms + 25) {
      Serial.print("Yaw: ");
      yawAngle = mpu.getYaw();
      Serial.print(yawAngle,2);
      Serial.print(" - prev_ms: ");
      prev_ms = millis();
      Serial.println(prev_ms);

      if (prev_ms > 15000 && prev_ms < 15100){
        initAngle = yawAngle;
        Serial.print("Initial Angle: ");
        Serial.print(yawAngle,2);
      }
    }
  }
}
