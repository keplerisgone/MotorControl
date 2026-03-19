#include <Arduino.h> // 아두이노 기본 라이브러리 포함

// 1. 핀 번호 정의
const int PIN_ENC_A = 2; // 엔코더 A상 (인터럽트 지원 핀)
const int PIN_ENC_B = 3; // 엔코더 B상 (방향 판별용)

// 2. 모터 및 엔코더 파라미터
const float GEAR_RATIO = 27.0; // 감속비 1/27
const float PPR = 26.0;        // 모터 축 1회전당 펄스 (Hall Sensor 기준)
const float DEGREES_PER_PULSE = 360.0 / (PPR * GEAR_RATIO);

// 3. 실시간 데이터를 저장할 변수
volatile long pulseCount = 0;  // 인터럽트에서 수정되므로 volatile 선언
long lastPulseCount = 0;       // 이전 펄스값 저장용

// 4. 엔코더 펄스 처리 함수 (ISR: Interrupt Service Routine)
void handleEncoder() {
    // A상이 Rising(0->5V)일 때 B상의 상태를 읽어 방향 판별
    // B가 LOW이면 정회전, HIGH이면 역회전
    if (digitalRead(PIN_ENC_B) == LOW) {
        pulseCount++;
    } else {
        pulseCount--;
    }
}

void setup() {
    // 시리얼 통신 시작 (보레이트 9600)
    Serial.begin(9600);
    
    // 핀 모드 설정 (내부 풀업 저항 사용하여 신호 안정화)
    pinMode(PIN_ENC_A, INPUT_PULLUP);
    pinMode(PIN_ENC_B, INPUT_PULLUP);
    
    // 인터럽트 설정: A상 핀의 전압이 상승(RISING)할 때 handleEncoder 실행
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), handleEncoder, RISING);
    
    Serial.println("================================");
    Serial.println("IG-30GM Encoder Test Started");
    Serial.println("Rotate the motor shaft by hand.");
    Serial.println("================================");
}

void loop() {
    // 값이 변했을 때만 출력하여 시리얼 모니터 부하 감소
    if (pulseCount != lastPulseCount) {
        // 현재 각도 계산
        float currentAngle = pulseCount * DEGREES_PER_PULSE;
        
        // 데이터 출력
        Serial.print("Pulse Count: ");
        Serial.print(pulseCount);
        Serial.print(" \t| Angle: ");
        Serial.print(currentAngle, 2); // 소수점 둘째자리까지
        Serial.println(" deg");
        
        lastPulseCount = pulseCount;
    }
    
    // CPU 부하를 줄이기 위한 아주 짧은 지연
    delay(10);
}