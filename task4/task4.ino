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

/*****MAIN PROGRAM*****/
void setup() {
  // put your setup code here, to run once:
  pinMode(PWMA,OUTPUT);
  pinMode(PWMB,OUTPUT);
  pinMode(AIN1,OUTPUT);
  pinMode(BIN1,OUTPUT);
  pinMode(STBY,OUTPUT);
  analogWrite(PWMA, 90);
  analogWrite(PWMB, 90);
}

void loop() {
  // put your main code here, to run repeatedly:
  forward();           //move forward for 138 cm
  delay(3600);         //stop at p2
  stop();
  delay(500);
  turnRight();         //turn left
  delay(600);         //stop at 90 deg
  stop();
  delay(500);
}
