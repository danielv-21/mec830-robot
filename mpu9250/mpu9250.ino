#include "MPU9250.h"

MPU9250 mpu;

float refAngle;
float yawAngle;
float relAngle;

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
  measureRelAngle();
}

float measureRelAngle(){
  if (mpu.update()) {
    static uint32_t prev_ms = millis();
    if (millis() > prev_ms + 25) {
      yawAngle = mpu.getYaw();
      prev_ms = millis();

      if (prev_ms > 15000 && prev_ms < 15100){          // wait for 15s to save initial angle
        refAngle = yawAngle;
      }

      relAngle = refAngle - yawAngle;
      Serial.print("Relative Angle ");
      Serial.println(relAngle, 2);
    }
  }
}
