#include <Arduino.h>
#include <PID_v1.h>

// ================= 핀 정의 =================
#define PIN_ENC_A 2
#define PIN_ENC_B 3
#define PIN_BTN_UP 5    // 정방향 수동 버튼
#define PIN_BTN_DOWN 6  // 역방향 수동 버튼
#define PIN_IN1 7       // 방향 1
#define PIN_IN2 8       // 방향 2
#define PIN_PWM 9       // 속도 제어

// ================= 시스템 상수 =================
const float GEAR_RATIO = 27.0;
const float PPR = 26.0; // 30GM - 26, 32GM - 512
const float MAX_LIMIT = 180.0; // 최대 회전각
const float MIN_LIMIT = 0.0;   // 최소 회전각

// ================= 전역 변수 =================
volatile long pulseCount = 0;
double Setpoint = 0, Input = 0, Output = 0;
double Kp = 2.0, Ki = 0.0, Kd = 0.0; // PID 초기 게인
bool isManual = false; // 수동 제어 모드 플래그

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// ================= 함수 선언 =================
void handleEncoder();
void driveMotor(double out);
void parseSerialCommand();

void setup() {
    Serial.begin(115200);
    
    // 핀 모드 설정
    pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT); pinMode(PIN_PWM, OUTPUT);
    pinMode(PIN_ENC_A, INPUT_PULLUP); pinMode(PIN_ENC_B, INPUT_PULLUP);
    pinMode(PIN_BTN_UP, INPUT_PULLUP); pinMode(PIN_BTN_DOWN, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), handleEncoder, RISING);

    // PID 설정
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(-200, 200); // 박스 보호를 위해 최대 출력 200으로 제한
    myPID.SetSampleTime(10);
}

void loop() {
    // 1. 현재 각도 계산
    Input = (pulseCount / (PPR * GEAR_RATIO)) * 360.0;

    // 2. 시리얼 입력 처리 (각도 및 PID 튜닝)
    parseSerialCommand();

    // 3. 버튼 수동 조작 및 PID 모드 전환
    if (digitalRead(PIN_BTN_UP) == LOW) {
        isManual = true;
        driveMotor(150);     // 버튼 누르면 정방향 150 출력
        Setpoint = Input;    // 떼었을 때 그 자리에 멈추도록 Setpoint 실시간 갱신
    } 
    else if (digitalRead(PIN_BTN_DOWN) == LOW) {
        isManual = true;
        driveMotor(-150);    // 버튼 누르면 역방향 150 출력
        Setpoint = Input;
    } 
    else {
        // 버튼을 떼면 PID 자동 제어 복귀
        if (isManual) isManual = false;
        
        myPID.Compute();     // PID 계산
        driveMotor(Output);  // 계산된 음수/양수 값으로 구동
    }

    // 4. 상태 모니터링 (플로터용)
    // Serial.print(Setpoint); Serial.print(" "); Serial.println(Input);
}

// ================= 핵심 제어 함수 =================

// 입력받은 Output(음수/양수)을 방향과 PWM으로 분리하고 안전 리미트를 적용하는 함수
void driveMotor(double out) {
    // [안전 장치] 물리적 한계를 넘으려는 힘은 즉시 0으로 차단 (수동/자동 모두 적용)
    if (Input >= MAX_LIMIT && out > 0) out = 0; 
    if (Input <= MIN_LIMIT && out < 0) out = 0;

    if (out > 0) {
        digitalWrite(PIN_IN1, HIGH);
        digitalWrite(PIN_IN2, LOW);
    } else if (out < 0) {
        digitalWrite(PIN_IN1, LOW);
        digitalWrite(PIN_IN2, HIGH);
    } else {
        digitalWrite(PIN_IN1, LOW);
        digitalWrite(PIN_IN2, LOW);
    }
    analogWrite(PIN_PWM, abs(out)); // 세기는 무조건 양수로 변환하여 출력
}

// 시리얼 명령어를 분석하여 Setpoint나 PID 게인을 수정하는 함수
void parseSerialCommand() {
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        float val = Serial.parseFloat();

        switch (cmd) {
            case 'S': case 's': 
                Setpoint = constrain(val, MIN_LIMIT, MAX_LIMIT); // 각도는 리미트 안으로 가둠
                break;
            case 'P': case 'p': Kp = val; myPID.SetTunings(Kp, Ki, Kd); break;
            case 'I': case 'i': Ki = val; myPID.SetTunings(Kp, Ki, Kd); break;
            case 'D': case 'd': Kd = val; myPID.SetTunings(Kp, Ki, Kd); break;
            default: return;
        }
        
        Serial.print("Update -> S:"); Serial.print(Setpoint);
        Serial.print(" | P:"); Serial.print(Kp);
        Serial.print(" I:"); Serial.print(Ki);
        Serial.print(" D:"); Serial.println(Kd);
    }
}

// 엔코더 펄스 카운트 (A상 RISING일 때 B상 상태로 방향 판단)
void handleEncoder() {
    if (digitalRead(PIN_ENC_B) == LOW) pulseCount++;
    else pulseCount--;
}