#include <IRremote.h>

int RECV_PIN = 9;

IRrecv irrecv(RECV_PIN);

decode_results results;

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
  analogWrite(PWMA, 50);
  analogWrite(PWMB, 50);
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
  Serial.begin(9600);
  
  pinMode(PWMA,OUTPUT);
  pinMode(PWMB,OUTPUT);
  pinMode(AIN1,OUTPUT);
  pinMode(BIN1,OUTPUT);
  pinMode(STBY,OUTPUT);

  Serial.println("Enabling IRin");
  irrecv.enableIRIn();
  Serial.println("Enabled IRin");
}

void loop() {
  // put your main code here, to run repeatedly:
  if (irrecv.decode(&results)){
    Serial.println(results.value);
    irrecv.resume();
  }
  delay(100);

  if (results.value == 16736925){
    forward();
  }
  else if (results.value == 16754775){
    backward();
  }
  else if (results.value == 16761405){
    turnRight();
  }
  else if (results.value == 16720605){
    turnLeft();
  }
  else if (results.value == 16712445){
    stop();
  }
  else{
  }
}
