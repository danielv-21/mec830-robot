int CLK = 2;
int DT = 3;
int SW = 4;

const int interrupt0 = 0;

int count = 0;

int lastCLK = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(SW, INPUT);
  digitalWrite(SW, HIGH);
  pinMode(CLK, INPUT);
  pinMode(DT, INPUT);

  attachInterrupt(interrupt0, ClockChanged, CHANGE);

  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(!digitalRead(SW) && count != 9){
    count = 0;
    Serial.print("count: ");
    Serial.println(count);
  }
}

void ClockChanged(){
  int clkValue = digitalRead(CLK);
  int dtValue = digitalRead(DT);

  if(lastCLK != clkValue){
    lastCLK != clkValue;

    count += (clkValue != dtValue ?1:-1);

    Serial.print("count: ");
    Serial.println(count);
  }
}
