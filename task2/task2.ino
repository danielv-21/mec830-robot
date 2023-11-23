#include <NewPing.h>
#include "MPU9250.h"
#include <Servo.h>

/*****Servo*****/
Servo myServo;

int servopin = 11;
int initpos = 90;

/*****MPU9250*****/
MPU9250 mpu;

float refAngle;
float yawAngle;
float relAngle;

/*****Define ultrasonic sensor pins*****/
#define trigger 13
#define echo 12
#define max_distance 200

NewPing sonar(trigger,echo,max_distance);

/*****Define pins used by motor driver*****/
#define PWMA 5
#define PWMB 6
#define STBY 3
#define AIN1 8
#define BIN1 7

/*****Motion objects*****/
void forward(){
  digitalWrite(STBY,HIGH);
  digitalWrite(AIN1,HIGH);
  digitalWrite(BIN1,HIGH);
  analogWrite(PWMA, 75);
  analogWrite(PWMB, 75);
}

void backward(){
  digitalWrite(STBY,HIGH);
  digitalWrite(AIN1,LOW);
  digitalWrite(BIN1,LOW); 
  analogWrite(PWMA, 50);
  analogWrite(PWMB, 50);
}

void stop(){
  digitalWrite(STBY,LOW);
}

void turnRight(){
  digitalWrite(STBY,HIGH);
  digitalWrite(AIN1,HIGH);
  digitalWrite(BIN1,LOW);
  analogWrite(PWMA, 50);
  analogWrite(PWMB, 50);
}

void turnLeft(){
  digitalWrite(STBY,HIGH);
  digitalWrite(AIN1,LOW);
  digitalWrite(BIN1,HIGH);
  analogWrite(PWMA, 50);
  analogWrite(PWMB, 50);
}

int count = 1;
float cm;

/*****MAIN PROGRAM*****/
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  /*****Initialize motor*****/
  pinMode(PWMA,OUTPUT);
  pinMode(PWMB,OUTPUT);
  pinMode(AIN1,OUTPUT);
  pinMode(BIN1,OUTPUT);
  pinMode(STBY,OUTPUT);

  /*****Initialize servo*****/
  myServo.attach(servopin);
  myServo.write(initpos);

  /*****Initialize MPU9250*****/
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
  // put your main code here, to run repeatedly:
  measureRelAngle();
  
  switch (count){
    case 1:
      if (refAngle == yawAngle){
        count = 2;
        break;
      }
    case 2:
      delay(5);
      cm = sonar.ping_cm();
//      Serial.print("Distance = ");
//      Serial.println(cm);
      
      if(cm >= 15){
        forward();
      }
      else if(cm <15){
        stop();
        count = 3;
        break;
      }
    case 3:
      Serial.print("RelAngle = ");
      Serial.print(relAngle);
      Serial.print("  -  Count = ");
      Serial.println(count);
  }
  
  
}

float measureRelAngle(){
  if (mpu.update()) {
    static uint32_t prev_ms = millis();
    if (millis() > prev_ms + 25) {
      yawAngle = mpu.getYaw();
      prev_ms = millis();

      if (prev_ms > 15000 && prev_ms < 15050){          // wait for 15s to save initial angle
        refAngle = yawAngle;
//        Serial.print("Reference Angle: ");
//        Serial.print(refAngle, 2);
      }

      relAngle = refAngle - yawAngle;
//      Serial.print("Relative Angle ");
//      Serial.println(relAngle,2);
    }
  }
}
