#include <Arduino.h>

int i = 0;

void setup(){
  Serial.begin(115200);
}
void loop(){
  i += 1;
  Serial.println(i);
  delay(10);

}