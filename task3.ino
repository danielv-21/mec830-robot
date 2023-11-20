#include <Servo.h>
#include <NewPing.h>

/*****Define ultrasonic sensor pins*****/
#define trigger 13
#define echo 12
#define max_distance 200

/*****Define servo pin*****/
int servopin = 11;
int i = 0;

Servo myServo;
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

void setup() {
  // put your setup code here, to run once:
  /*****initialize motor driver pins*****/
  pinMode(PWMA,OUTPUT);
  pinMode(PWMB,OUTPUT);
  pinMode(AIN1,OUTPUT);
  pinMode(BIN1,OUTPUT);
  pinMode(STBY,OUTPUT);
  analogWrite(PWMA, 90);
  analogWrite(PWMB, 90);
  
  Serial.begin(9600);
  myServo.attach(servopin);
}

void loop() {
  // put your main code here, to run repeatedly:
  for (i=0;i<=180;i++){
    myServo.write(i);
    forward();
    delay(10);
    Serial.print("i = ");
    Serial.print(i);
    
    int cm = sonar.ping_cm();          //ping_cm() returns integer
    Serial.print(" - Ping: ");
    Serial.println(cm);

    if (cm == 10){
      turnLeft();
      delay(600);                       //stop at 90 deg
    }
  }
  for (i=180;i>=0;i--){
    myServo.write(i);
    forward();
    delay(10);
    Serial.print("i = ");
    Serial.print(i);

    int cm = sonar.ping_cm();          //ping_cm() returns integer
    Serial.print(" - Ping: ");
    Serial.println(cm);
  }
}
