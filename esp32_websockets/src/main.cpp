#include <Arduino.h>
#include<WiFi.h>
#include<ArduinoWebsockets.h>
#include<ArduinoJson.h>

const char* ssid = "Sanath kumar";
const char* password = "sanath2006";
const char* websocket_server = "192.168.0.10";
const int websocket_port = 8765;
unsigned long lastcmmndtime = 0;
const unsigned long command_timeout = 500;

#define ENCODER_A 18  // Left Motor Encoder
#define ENCODER_B 19  // Right Motor Encoder

#define ENA  13  // Speed Control
#define IN1  12  // Direction Control
#define IN2  14  


// Motor B (Right Motor)
#define ENB  27  // Speed Control
#define IN3  26  // Direction Control
#define IN4  25  
float x=0, z=0;
volatile int ticks_A = 0, ticks_B = 0;
// #define ENA  13  // Speed Control
// #define IN1  12  // Direction Control
// #define IN2  14  

// // Motor B (Right Motor)
// #define ENB  27  // Speed Control
// #define IN3  26  // Direction Control
// #define IN4  25  
bool is_moving = false;
void IRAM_ATTR encoderA_ISR() {
  ticks_A++;
}

void IRAM_ATTR encoderB_ISR() {
  ticks_B++;
}

void attachEncoders() {
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderA_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), encoderB_ISR, RISING);
}

void detachEncoders() {
  detachInterrupt(digitalPinToInterrupt(ENCODER_A));
  detachInterrupt(digitalPinToInterrupt(ENCODER_B));
}
class MotorControl{
    private:
       int eni, inta, intb;//left motor
       int enj, intc, intd;//right motor
       
       int speed = 0;

    public:
    static MotorControl *instance;
    //constructor
       MotorControl(int ena, int in1, int in2, int enb, int in3, int in4){
          //can also write like this "this->eni = ena;""
          eni = ena;
          inta = in1;
          intb = in2;
          enj = enb;
          intc = in3;
          intd = in4;
          

       }
       void appspeed(){
        if (speed>0){
        ledcWrite(0, speed);
        ledcWrite(1, speed);
        }
    }
    void prep_motion(){
      if (!is_moving) {
        ticks_A = 0;
        ticks_B = 0;
        attachEncoders();   // attach only ONCE when starting motion
        is_moving = true;
      }
    }

       void begin(int speed1){
     
        pinMode(eni, OUTPUT);
        pinMode(enj, OUTPUT);
        pinMode(inta, OUTPUT);
        pinMode(intb, OUTPUT);
        pinMode(intc, OUTPUT);
        pinMode(intd, OUTPUT);
        ledcSetup(0,1000,8);
        ledcSetup(1,1000,8);
        ledcAttachPin(eni,0);
        ledcAttachPin(enj,1);
        speed = speed1;
        appspeed();
        }
      void stop(){
        if (is_moving) {
          
          detachEncoders();   // attach only ONCE when starting motion
          is_moving = false;
        }
        ledcWrite(0, 0);  // Stop left motor PWM
        ledcWrite(1, 0);  // Stop right motor PWM
        
        
          digitalWrite(inta, LOW); digitalWrite(intb, LOW);
          digitalWrite(intc, LOW); digitalWrite(intd, LOW);
          detachEncoders();
        }
      void forward(){
        prep_motion();

        appspeed();
        digitalWrite(inta, HIGH); digitalWrite(intb, LOW);
        digitalWrite(intc, HIGH); digitalWrite(intd, LOW);


      }
      void back(){
        prep_motion();
        appspeed();
        digitalWrite(inta, LOW); digitalWrite(intb, HIGH);
        digitalWrite(intc, LOW); digitalWrite(intd, HIGH);

        
      }
      void left(){
        prep_motion();
        
        appspeed();
        digitalWrite(inta, LOW); digitalWrite(intb, HIGH);
        digitalWrite(intc, HIGH); digitalWrite(intd, LOW);
     
      }
      void right(){
        prep_motion();
        appspeed();
        digitalWrite(inta, HIGH); digitalWrite(intb, LOW);
        digitalWrite(intc, LOW); digitalWrite(intd, HIGH);
  
      }


}; 
MotorControl *MotorControl::instance = nullptr;
MotorControl motors(13,12,14,27,26,25);

using namespace websockets;
WebsocketsClient client;



void onMessageCallback(WebsocketsMessage message){

  Serial.println("Received: "+ message.data());
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, message.data());
  if (error){
    Serial.println("json parsing failed!!!");
    return;
  }   
  x = doc["linear_x"];
  z = doc["angular_z"];

  Serial.printf("linear_x: %.2f, angular_z: %.2f \n", x, z);
  lastcmmndtime = millis();

  #include <Arduino.h>
  #include<WiFi.h>
  #include<ArduinoWebsockets.h>
  #include<ArduinoJson.h>
  
  const char* ssid = "Sanath kumar";
  const char* password = "sanath2006";
  const char* websocket_server = "192.168.0.10";
  const int websocket_port = 8765;
  unsigned long lastcmmndtime = 0;
  const unsigned long command_timeout = 500;
  
  #define ENCODER_A 18  // Left Motor Encoder
  #define ENCODER_B 19  // Right Motor Encoder
  
  #define ENA  13  // Speed Control
  #define IN1  12  // Direction Control
  #define IN2  14  
  
  
  // Motor B (Right Motor)
  #define ENB  27  // Speed Control
  #define IN3  26  // Direction Control
  #define IN4  25  
  float x=0, z=0;
  volatile int ticks_A = 0, ticks_B = 0;
  // #define ENA  13  // Speed Control
  // #define IN1  12  // Direction Control
  // #define IN2  14  
  
  // // Motor B (Right Motor)
  // #define ENB  27  // Speed Control
  // #define IN3  26  // Direction Control
  // #define IN4  25  
  bool is_moving = false;
  void IRAM_ATTR encoderA_ISR() {
    ticks_A++;
  }
  
  void IRAM_ATTR encoderB_ISR() {
    ticks_B++;
  }
  
  void attachEncoders() {
    attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderA_ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_B), encoderB_ISR, RISING);
  }
  
  void detachEncoders() {
    detachInterrupt(digitalPinToInterrupt(ENCODER_A));
    detachInterrupt(digitalPinToInterrupt(ENCODER_B));
  }
  class MotorControl{
      private:
         int eni, inta, intb;//left motor
         int enj, intc, intd;//right motor
         
         int speed = 0;
  
      public:
      static MotorControl *instance;
      //constructor
         MotorControl(int ena, int in1, int in2, int enb, int in3, int in4){
            //can also write like this "this->eni = ena;""
            eni = ena;
            inta = in1;
            intb = in2;
            enj = enb;
            intc = in3;
            intd = in4;
            
  
         }
         void appspeed(){
          if (speed>0){
          ledcWrite(0, speed);
          ledcWrite(1, speed);
          }
      }
      void prep_motion(){
        if (!is_moving) {
          ticks_A = 0;
          ticks_B = 0;
          attachEncoders();   // attach only ONCE when starting motion
          is_moving = true;
        }
      }
  
         void begin(int speed1){
       
          pinMode(eni, OUTPUT);
          pinMode(enj, OUTPUT);
          pinMode(inta, OUTPUT);
          pinMode(intb, OUTPUT);
          pinMode(intc, OUTPUT);
          pinMode(intd, OUTPUT);
          ledcSetup(0,1000,8);
          ledcSetup(1,1000,8);
          ledcAttachPin(eni,0);
          ledcAttachPin(enj,1);
          speed = speed1;
          appspeed();
          }
        void stop(){
          if (is_moving) {
            
            detachEncoders();   // attach only ONCE when starting motion
            is_moving = false;
          }
          ledcWrite(0, 0);  // Stop left motor PWM
          ledcWrite(1, 0);  // Stop right motor PWM
          
          
            digitalWrite(inta, LOW); digitalWrite(intb, LOW);
            digitalWrite(intc, LOW); digitalWrite(intd, LOW);
            detachEncoders();
          }
        void forward(){
          prep_motion();
  
          appspeed();
          digitalWrite(inta, HIGH); digitalWrite(intb, LOW);
          digitalWrite(intc, HIGH); digitalWrite(intd, LOW);
  
  
        }
        void back(){
          prep_motion();
          appspeed();
          digitalWrite(inta, LOW); digitalWrite(intb, HIGH);
          digitalWrite(intc, LOW); digitalWrite(intd, HIGH);
  
          
        }
        void left(){
          prep_motion();
          
          appspeed();
          digitalWrite(inta, LOW); digitalWrite(intb, HIGH);
          digitalWrite(intc, HIGH); digitalWrite(intd, LOW);
       
        }
        void right(){
          prep_motion();
          appspeed();
          digitalWrite(inta, HIGH); digitalWrite(intb, LOW);
          digitalWrite(intc, LOW); digitalWrite(intd, HIGH);
    
        }
  
  
  }; 
  MotorControl *MotorControl::instance = nullptr;
  MotorControl motors(13,12,14,27,26,25);
  
  using namespace websockets;
  WebsocketsClient client;
  
  
  
  void onMessageCallback(WebsocketsMessage message){
  
    Serial.println("Received: "+ message.data());
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, message.data());
    if (error){
      Serial.println("json parsing failed!!!");
      return;
    }   
    x = doc["linear_x"];
    z = doc["angular_z"];
  
    Serial.printf("linear_x: %.2f, angular_z: %.2f \n", x, z);
    lastcmmndtime = millis();
  
  
  }
  
  
  void setup(){
    pinMode(2, OUTPUT);  // Built-in LED
    delay(1000); 
    Serial.begin(115200);
    delay(1000); 
        // Motor Setup
        pinMode(ENA, OUTPUT);
        pinMode(IN1, OUTPUT);
        pinMode(IN2, OUTPUT);
        
        pinMode(ENB, OUTPUT);
        pinMode(IN3, OUTPUT);
        pinMode(IN4, OUTPUT);
    
        // PWM Setup (ESP32 uses LEDC for PWM)
        ledcSetup(0, 1000, 8); // Channel 0, 1 kHz freq, 8-bit resolution
        ledcAttachPin(ENA, 0);
        
        ledcSetup(1, 1000, 8);
        ledcAttachPin(ENB, 1);
  
        motors.begin(100);
        WiFi.begin(ssid,password);
  
  
    while (WiFi.status() != WL_CONNECTED){
      delay(500);
      Serial.print(".");
      digitalWrite(2, HIGH);
      delay(100);
      digitalWrite(2, LOW);
      delay(100);
  
    }
    digitalWrite(2, HIGH);  // Connected
  
   
    Serial.println("\n Connected to Wifi!!!");
    Serial.println(WiFi.localIP());
    delay(2000);
    client.onMessage(onMessageCallback);
    if (client.connect("ws://192.168.0.10:8765")){
      Serial.println("Connected to Websocket Server!!!");
    }
    if (!client.connect("ws://192.168.0.10:8765")) {
      Serial.println("WebSocket connection failed!");
      while (true) {
        delay(500);
        Serial.print(".");
      }  // Halt execution or retry logic.
  }
  
    // attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderA_ISR, RISING);
    // attachInterrupt(digitalPinToInterrupt(ENCODER_B), encoderB_ISR, RISING);
  
  }
  void forward2(){
  
    if (x>0){
      Serial.println("Moving_Forward!!!");
  
      motors.forward();
    } else if (x<0)
    {
      Serial.println("Moving_Backward");
      motors.back();
    }
    else if (z>0)
    {
      Serial.println("Turning_left");
      motors.left();  
    }else if (z<0)
    {
      Serial.println("Turning_right");
      motors.right();
    } 
    else if(x== 0 && z == 0){
      motors.stop();
    }
  }
  void loop(){
    forward2();
    if(millis()- lastcmmndtime > command_timeout){
      motors.stop();
    }
    if(client.available()){
      JsonDocument doc;
      doc["left_tick"] = ticks_A;
      doc["right_ticks"] = ticks_B;
      String jsonData;
      serializeJson(doc, jsonData);
      client.send(jsonData);
    }
    client.poll();
  
  }
}


void setup(){
  pinMode(2, OUTPUT);  // Built-in LED
  delay(1000); 
  Serial.begin(115200);
  delay(1000); 
      // Motor Setup
      pinMode(ENA, OUTPUT);
      pinMode(IN1, OUTPUT);
      pinMode(IN2, OUTPUT);
      
      pinMode(ENB, OUTPUT);
      pinMode(IN3, OUTPUT);
      pinMode(IN4, OUTPUT);
  
      // PWM Setup (ESP32 uses LEDC for PWM)
      ledcSetup(0, 1000, 8); // Channel 0, 1 kHz freq, 8-bit resolution
      ledcAttachPin(ENA, 0);
      
      ledcSetup(1, 1000, 8);
      ledcAttachPin(ENB, 1);

      motors.begin(100);
      WiFi.begin(ssid,password);


  while (WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(".");
    digitalWrite(2, HIGH);
    delay(100);
    digitalWrite(2, LOW);
    delay(100);

  }
  digitalWrite(2, HIGH);  // Connected

 
  Serial.println("\n Connected to Wifi!!!");
  Serial.println(WiFi.localIP());
  delay(2000);
  client.onMessage(onMessageCallback);
  if (client.connect("ws://192.168.0.10:8765")){
    Serial.println("Connected to Websocket Server!!!");
  }
  if (!client.connect("ws://192.168.0.10:8765")) {
    Serial.println("WebSocket connection failed!");
    while (true) {
      delay(500);
      Serial.print(".");
    }  // Halt execution or retry logic.
}

  // attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderA_ISR, RISING);
  // attachInterrupt(digitalPinToInterrupt(ENCODER_B), encoderB_ISR, RISING);

}
void forward2(){

  if (x>0){
    Serial.println("Moving_Forward!!!");

    motors.forward();
  } else if (x<0)
  {
    Serial.println("Moving_Backward");
    motors.back();
  }
  else if (z>0)
  {
    Serial.println("Turning_left");
    motors.left();  
  }else if (z<0)
  {
    Serial.println("Turning_right");
    motors.right();
  } 
  else if(x== 0 && z == 0){
    motors.stop();
  }
}
void loop(){
  forward2();
  if(millis()- lastcmmndtime > command_timeout){
    motors.stop();
  }
  if(client.available()){
    JsonDocument doc;
    doc["left_tick"] = ticks_A;
    doc["right_ticks"] = ticks_B;
    String jsonData;
    serializeJson(doc, jsonData);
    client.send(jsonData);
  }
  client.poll();

}