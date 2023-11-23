#include <NewPing.h>
#include <Servo.h>
#include <Wire.h>

/*****MPU9250*****/
const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ; //linear acceleration
float GyroX, GyroY, GyroZ; //angular velocity
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ; //used in void loop()
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;

float AngleRoll, AnglePitch;
float RateRoll, RatePitch, RateYaw;

float RateCalibrationRoll,RateCalibrationPitch, RateCalibrationYaw; 
int RateCalibrationNumber;

float angle;

/*****Kalman Filter*****/
float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2*2;          //Define the predicted angles and the uncertainties
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2*2;

float Kalman1DOutput[] = {0,0};         //Initialize the output of the filter {angle prediction, prediction undertainty}


/*****Servo*****/
Servo myServo;

int servopin = 11;
int initpos = 90;

/*****Define ultrasonic sensor pins*****/
#define trigger 13
#define echo 12
#define max_distance 200

float cm;

NewPing sonar(trigger,echo,max_distance);

//Create the function that calculates the predicted angle and uncertainty using the Kalman equations
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement){
  KalmanState = KalmanState + 0.004 * KalmanInput;
  KalmanUncertainty = KalmanUncertainty + 0.004 * 0.008 * 4 * 4;

  float KalmanGain = KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3);
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty = (1-KalmanGain)*KalmanUncertainty;

  Kalman1DOutput[0] = KalmanState;          //Kalman filter output
  Kalman1DOutput[1] = KalmanUncertainty;
}

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

  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  // Call this function if you need to get the IMU error values for your module
  calculateError();
  delay(20);

  for(RateCalibrationNumber = 0;
      RateCalibrationNumber < 2000;
      RateCalibrationNumber++){
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    delay(1);
  }
  RateCalibrationRoll /= 2000;
  RateCalibrationPitch /= 2000;
  RateCalibrationYaw /= 2000;

  currentTime = micros();
}

int count = 1;

void loop() {
  // put your main code here, to run repeatedly:
  // === Read accelerometer (on the MPU6050) data === //
  readAcceleration();
  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorX; //AccErrorX is calculated in the calculateError() function
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorY;
  
  // === Read gyroscope (on the MPU6050) data === //
  previousTime = currentTime;
  currentTime = micros();
  elapsedTime = (currentTime - previousTime) / 1000000; // Divide by 1000 to get seconds
  readGyro();
  // Correct the outputs with the calculated error values
  GyroX -= GyroErrorX; //GyroErrorX is calculated in the calculateError() function
  GyroY -= GyroErrorY;
  GyroZ -= GyroErrorZ;
  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX += GyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY += GyroY * elapsedTime;
  yaw += GyroZ * elapsedTime;
  //combine accelerometer- and gyro-estimated angle values. 0.96 and 0.04 values are determined through trial and error by other people
  roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
  angle = yaw; //if you mounted MPU6050 in a different orientation to me, angle may not = roll. It can roll, pitch, yaw or minus version of the three
  //for me, turning right reduces angle. Turning left increases angle.

  //Start the iteration for the Kalman filter with the roll and pitch angles
  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll = Kalman1DOutput[0]+0.005;
  KalmanUncertaintyAngleRoll = Kalman1DOutput[1];

  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch = Kalman1DOutput[0];
  KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

  Serial.print("Yaw: ");
  Serial.print(yaw);
  
  cm = sonar.ping_cm();
  Serial.print(" - cm: ");
  Serial.print(cm);

  Serial.print(" - Count: ");
  Serial.println(count);

  switch (count){
    case 1:   // p0 to p1
      while (cm >= 15){
        forward();
      }
      stop();
      turnRight();
      count = 2;
      break;
      
//    case 2:   //turn right
//      if (yaw >= -90){
//        turnRight();
//      }
//      else if (cm < -90){
//        stop();
//        count = 3;
//      }
//    case 3:   //p1 to p2
//      if (cm >= 15){
//        forward();
//      }
//      else if (cm < 15){
//        stop();
//        count = 4;
//      }
//    case 4:   //turn left
//      if (yaw <= 90){
//        turnLeft();
//      }
//      else if (yaw > 90){
//        stop();
//        count = 5;
//      }
//    case 5:   //p2 to p3
//      if (cm >= 15){
//        forward();
//      }
//      else if (cm < 15){
//        stop();
//      }
  }
  

}

void calculateError() {
  //When this function is called, ensure the car is stationary. See Step 2 for more info
  
  // Read accelerometer values 200 times
  c = 0;
  while (c < 200) {
    readAcceleration();
    // Sum all readings
    AccErrorX += (atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI);
    AccErrorY += (atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI);
    c++;
  }
  //Divide the sum by 200 to get the error value, since expected value of reading is zero
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;
  
  // Read gyro values 200 times
  while (c < 200) {
    readGyro();
    // Sum all readings
    GyroErrorX += GyroX;
    GyroErrorY += GyroY;
    GyroErrorZ += GyroZ;
    c++;
  }
  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
  Serial.println("The the gryoscope setting in MPU6050 has been calibrated");
}

void readAcceleration() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the MPU6050 datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
}

void readGyro() {
  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
}
