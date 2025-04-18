#include <Arduino.h>

void setup(){
  Serial.begin(115200);
}
void loop(){
  if (Serial.available()>0){
    String received_data = Serial.readStringUntil('\n');
    Serial.print("Received: ");
    Serial.println(received_data.toInt()+1);
     
  }

}