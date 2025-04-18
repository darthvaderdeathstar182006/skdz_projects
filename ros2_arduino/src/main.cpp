#include <Arduino.h>
int i = 0;

void setup(){
  Serial.begin(115200);
}
void loop(){
  
    Serial.println(String(i));
    i += 1;
    delay(10);


}