#include <Servo.h>
#include <NewPing.h>

/*****Define ultrasonic sensor pins*****/
#define trigger 13
#define echo 12
#define max_distance 200

/*****Define servo pin*****/
int servopin = 11;
int i = 0;

int cm;
int count = 0;

/*****Define objects*****/
Servo myServo;
NewPing sonar(trigger,echo,max_distance);

/*****Define pins used by motor driver*****/
#define PWMA 5          // left motor speed
#define PWMB 6          // right motor speed
#define STBY 3          // enables/disables motors
#define AIN1 8          // two left motors
#define BIN1 7          // two right motors

/*****Motion functions*****/
void forward(){
  digitalWrite(STBY,HIGH);
  digitalWrite(AIN1,HIGH);
  digitalWrite(BIN1,HIGH);
}

void backward(){
  digitalWrite(STBY,HIGH);
  digitalWrite(AIN1,LOW);
  digitalWrite(BIN1,LOW); 
}

void stop(){
  digitalWrite(STBY,LOW);
}

void turnRight(){
  digitalWrite(STBY,HIGH);
  digitalWrite(AIN1,HIGH);
  digitalWrite(BIN1,LOW);
}

void turnLeft(){
  digitalWrite(STBY,HIGH);
  digitalWrite(AIN1,LOW);
  digitalWrite(BIN1,HIGH);
}

void defaultMotion(){
  myServo.write(90);              // ultrasonic sensor facing front
  forward();                      // car moves forward
}

void setup() {
  // put your setup code here, to run once:
  /*****initialize motor driver pins*****/
  pinMode(PWMA,OUTPUT);
  pinMode(PWMB,OUTPUT);
  pinMode(AIN1,OUTPUT);
  pinMode(BIN1,OUTPUT);
  pinMode(STBY,OUTPUT);
  analogWrite(PWMA, 90);          // left motor speed
  analogWrite(PWMB, 90);          // right motor speed
  
  Serial.begin(9600);
  myServo.attach(servopin);       // initialize servo

  defaultMotion();                // move forward with ultrasonic sensor facing front, no sweeping
  cm = sonar.ping_cm();

  while(cm > 15){
    defaultMotion();                // move forward with ultrasonic sensor facing front, no sweeping
    cm = sonar.ping_cm();           // ultrasonic sensor checks for obstacle
  }

  stop();                           // stop for 50ms when obstacle detected
  delay(50);
  turnLeft();
  delay(600);
  stop();
  count++;
}

void loop() {
  // put your main code here, to run repeatedly:

  switch (count){
    case 1:
      // Sweep 90 deg right to front to read distance
      int cmArray[91];
      for (i=0; i<=90;i++){
        myServo.write(i);
        cmArray[i] = sonar.ping_cm();
        delay(10);
        
        if (cmArray[0] <= 10){
          count = 2;
        }
        else if (cmArray[0] > 20){
          count = 3;
        }
        
      }
    case 2:
      // after case 1, move forward for 10cm
      defaultMotion();
      delay(1000);                    // not exactly 10cm, need to modify
      stop();
      count = 1;
    case 3:
      // if obstacle detected, turn right
      turnRight();
      delay(600);
      count = 1;
  }
  

//  for (i=0;i<=180;i++){               // servo sweep right to left
//    myServo.write(i);
//    delay(10);
////    Serial.print("i = ");
////    Serial.print(i);
//    
//    cm = sonar.ping_cm();          //ping_cm() returns integer
////    Serial.print(" - Ping: ");
////    Serial.println(cm);
//
///*****Create 1D array of distances*****/
//    int distance[181];
//    distance[i] = cm;
//  }
//  for (i=180;i>=0;i--){                 // servo sweep left to right
//    myServo.write(i);
//    delay(10);
////    Serial.print("i = ");
////    Serial.print(i);
//
//    cm = sonar.ping_cm();          //ping_cm() returns integer
////    Serial.print(" - Ping: ");
////    Serial.println(cm);
//
///*****Create 1D array of distances*****/
//    int distance[181];
//    distance[i] = cm;
//  }
}
