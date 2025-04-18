// Define motor control pins
#include<Arduino.h>
// Motor A (Left Motor)
#define ENA  13  // Speed Control
#define IN1  12  // Direction Control
#define IN2  14  

// Motor B (Right Motor)
#define ENB  27  // Speed Control
#define IN3  26  // Direction Control
#define IN4  25  

// Encoder Pins
#define ENCODER_A 18  // Left Motor Encoder
#define ENCODER_B 19  // Right Motor Encoder

volatile int ticks_A = 0;
volatile int ticks_B = 0;

// Interrupt Service Routines (ISRs) for encoders
void IRAM_ATTR encoderA_ISR() {
    ticks_A++;
}

void IRAM_ATTR encoderB_ISR() {
    ticks_B++;
}

// Function to drive motors
void driveMotors(int speedA, int speedB) {
    // Left Motor (Motor A) Forward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    ledcWrite(0, speedA); // Channel 0 for ENA

    // Right Motor (Motor B) Forward
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    ledcWrite(1, speedB); // Channel 1 for ENB
}

void setup() {
    Serial.begin(115200);

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

    // Encoder Setup
    pinMode(ENCODER_A, INPUT_PULLUP);
    pinMode(ENCODER_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderA_ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_B), encoderB_ISR, RISING);

    // Start Motors at speed 150
    driveMotors(150, 150);
}

void loop() {
    Serial.print("Left Motor Ticks: ");
    Serial.print(ticks_A);
    Serial.print(" | Right Motor Ticks: ");
    Serial.println(ticks_B);

    delay(500);  // Adjust delay as needed
}