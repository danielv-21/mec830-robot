#include <NewPing.h>
#include <Servo.h>
#include <Wire.h>

/*****MPU9250*****/
float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll,RateCalibrationPitch, RateCalibrationYaw; 
int RateCalibrationNumber;
float AccX, AccY, AccZ;         //Define the accelerometer variables
float AngleRoll, AnglePitch, AngleYaw;
float Xoffset=+0.07, Yoffset=0, Zoffset=+0.06;
float LoopTimer;

/*****Servo*****/
Servo myServo;

int servopin = 11;
int initpos = 90;

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

  Wire.setClock(400000);          //Communication with the gyroscope and calibration 
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  for(RateCalibrationNumber = 0;
      RateCalibrationNumber < 2000;
      RateCalibrationNumber++){
    gyro_signals();
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    delay(1);
  }
  RateCalibrationRoll /= 2000;
  RateCalibrationPitch /= 2000;
  RateCalibrationYaw /= 2000;
  LoopTimer=micros();
}

void loop() {
  // put your main code here, to run repeatedly:
  gyro_signals();

//  Serial.print("Acceleration X [g]- ");         //Check the measured acceleration values
//  Serial.print(AccX);
//  Serial.print(" Acceleration Y [g]- ");
//  Serial.print(AccY);
//  Serial.print(" Acceleration Z [g]- ");
//  Serial.println(AccZ);

  Serial.print("Yaw angle [°]= ");         //Check the measured roll and pitch angles
  Serial.print(AngleYaw);
  Serial.print(" Roll angle [°]= ");         
  Serial.print(AngleRoll);
  Serial.print(" Pitch angle [°]= ");
  Serial.println(AnglePitch);
}

void gyro_signals(void){
  Wire.beginTransmission(0x68);         //Switch on the low-pass filter
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);         //Configure the accelerometer output
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);         //Pull the accelerometer measurements from the sensor
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t AccXLSB = Wire.read()<<8 | Wire.read();
  int16_t AccYLSB = Wire.read()<<8 | Wire.read();
  int16_t AccZLSB = Wire.read()<<8 | Wire.read();

  Wire.beginTransmission(0x68);         //Configure the gyroscope output and pull rotation rate measurements from the sensor
  Wire.write(0x1B);
  Wire.write(0x8);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t GyroX = Wire.read()<<8 | Wire.read();
  int16_t GyroY = Wire.read()<<8 | Wire.read();
  int16_t GyroZ = Wire.read()<<8 | Wire.read();
  RateRoll = (float)GyroX/65.5;
  RatePitch = (float)GyroY/65.5;
  RateYaw = (float)GyroZ/65.5;

  // x: right is -, left is +
  // y: front is -, back is +
  // z: up is -, down is +
  AccX = (float)AccXLSB/4096+Xoffset;         //Convert the measurements to physical values
  AccY = (float)AccYLSB/4096+Yoffset;         //Zero offset calibration can be done here
  AccZ = (float)AccZLSB/4096+Zoffset;

  //Calculate the absolute angles
  AngleRoll = atan(AccY / sqrt(AccX*AccX + AccZ*AccZ)) * 1/(3.142/180);         
  AnglePitch = atan(AccX / sqrt(AccY*AccY + AccZ*AccZ)) * 1/(3.142/180);
  //AngleYaw = atan(sqrt(AccY*AccY + AccX*AccX) / AccZ) * 1/(3.142/180);
  AngleYaw = atan(-AccX / sqrt(AccY*AccY + AccZ*AccZ)) * 1/(3.142/180);
}
