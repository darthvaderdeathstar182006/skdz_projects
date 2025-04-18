#include <Arduino.h>
#include <NewPing.h>
#define trig 10
#define echo 13
#define maxd 400

NewPing sonar (trig,echo,maxd);
float distance, duration;

void setup(){
  Serial.begin(9600);
}
void loop(){
  duration = sonar.ping();
  distance = (duration/2)*0.0343;
  Serial.print("distance = ");
  if (distance >= 400 || distance<=2){
    Serial.println("out of range");
  }
  else{
     Serial.println(distance);
     Serial.println("cm");
     delay(500);
  }
  delay(500);
}
