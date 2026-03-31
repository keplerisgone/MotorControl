#include <Arduino.h>

// pinout
const int PIN_ENC_A = 2; // encoder A pin
const int PIN_ENC_B = 3; // encoder B pin

// Motor and encoder parameter
const float GEAR_RATIO = 27.0; 
const float PPR = 26.0;        
const float DEGREES_PER_PULSE = 360.0 / (PPR * GEAR_RATIO);

// count variable
volatile long pulseCount = 0;  
long lastPulseCount = 0;      

// Pulse Handler
void handleEncoder() {
    if (digitalRead(PIN_ENC_B) == LOW) {
        pulseCount++;
    } else {
        pulseCount--;
    }
}

void setup() {
    // Serial setup
    Serial.begin(9600);
    
    // Pin setup
    pinMode(PIN_ENC_A, INPUT_PULLUP);
    pinMode(PIN_ENC_B, INPUT_PULLUP);
    
    // interrupt setup
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), handleEncoder, RISING);
    
    Serial.println("================================");
    Serial.println("IG-30GM Encoder Test Started");
    Serial.println("Rotate the motor shaft by hand.");
    Serial.println("================================");
}

void loop() {
    // Print result only when the pulseCount changes
    if (pulseCount != lastPulseCount) {
        // calculate current angle
        float currentAngle = pulseCount * DEGREES_PER_PULSE;
        
        // Print data
        Serial.print("Pulse Count: ");
        Serial.print(pulseCount);
        Serial.print(" \t| Angle: ");
        Serial.print(currentAngle, 2); 
        Serial.println(" deg");
        
        lastPulseCount = pulseCount;
    }
    
    delay(10);
}