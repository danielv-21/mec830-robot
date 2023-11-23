#include <NewPing.h>

/*****Define ultrasonic sensor pins*****/
#define trigger 13
#define echo 12
#define max_distance 200

NewPing sonar(trigger,echo,max_distance);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(5);
  int cm = sonar.ping_cm();          //ping_cm() returns integer
  Serial.print("Ping: ");
  Serial.println(cm);
}
