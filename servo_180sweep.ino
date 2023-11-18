#include <Servo.h>

int servopin = 10;
int i = 0;

Servo myServo;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  myServo.attach(servopin);
}

void loop() {
  // put your main code here, to run repeatedly:
  for (i=0;i<=180;i++){
    myServo.write(i);
    delay(10);
    Serial.println(i);
  }
  for (i=180;i>=0;i--){
    myServo.write(i);
    delay(10);
    Serial.println(i);
  }
}
