#include <Arduino.h>

void setup() {
    // 시리얼 통신 속도 설정
    Serial.begin(9600);
    
    // 아두이노 Uno의 내장 LED(13번 핀)를 출력으로 설정
    pinMode(LED_BUILTIN, OUTPUT);
    
    Serial.println("--- LED Serial Control Test ---");
    Serial.println("Enter '1' to turn ON, '0' to turn OFF");
}

void loop() {
    // 시리얼 버퍼에 데이터가 들어왔는지 확인
    if (Serial.available() > 0) {
        // 문자를 읽어옴
        char command = Serial.read();
        
        if (command == '1') {
            digitalWrite(LED_BUILTIN, HIGH);
            Serial.println(">> LED: ON");
        } 
        else if (command == '0') {
            digitalWrite(LED_BUILTIN, LOW);
            Serial.println(">> LED: OFF");
        }
    }
}