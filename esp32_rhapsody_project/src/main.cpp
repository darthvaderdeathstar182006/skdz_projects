#include <Arduino.h>
#include<ESP32Servo.h>
Servo serv1;
Servo serv2;
Servo serv3;
Servo serv4;
Servo serv5;

void Setup(){
    Serial.begin(115200);
    serv1.attach(4);
    serv2.attach(5);
    serv3.attach(6);
    serv4.attach(7);
    serv5.attach(8);
    
}