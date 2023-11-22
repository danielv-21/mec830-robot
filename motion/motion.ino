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
  pinMode(PWMA,OUTPUT);
  pinMode(PWMB,OUTPUT);
  pinMode(AIN1,OUTPUT);
  pinMode(BIN1,OUTPUT);
  pinMode(STBY,OUTPUT);
  analogWrite(PWMA, 100);
  analogWrite(PWMB, 100);
}

void loop() {
  // put your main code here, to run repeatedly:
//  forward();
//  delay(1000);
  stop();
  delay(1000);
//  backward();
//  delay(1000);
//  turnLeft();
//  delay(3000);
  turnRight();
  delay(3000);
}
