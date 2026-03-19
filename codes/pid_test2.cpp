#include <Arduino.h>
#include <PID_v1.h>

// ================= 핀 정의 =================
#define PIN_ENC_A 2
#define PIN_ENC_B 3
#define PIN_IN1 7       
#define PIN_IN2 8       
#define PIN_PWM 9       

// ================= 시스템 상수 =================
const float GEAR_RATIO = 27.0;
const float PPR = 26.0; 
const float MAX_LIMIT = 180.0; // todo - limit을 어느정도로 설정해야 좋은지?
const float MIN_LIMIT = 0.0;   

// ================= 모드 정의 =================
enum ControlMode { PID_AUTO, MANUAL_JOG };
ControlMode currentMode = PID_AUTO; // 기본값은 PID 모드

// ================= 전역 변수 =================
volatile long pulseCount = 0;
double Setpoint = 0, Input = 0, Output = 0;
double Kp = 2.0, Ki = 0.0, Kd = 0.0; 
double manualSpeed = 0; // 수동 모드 시 출력 값 (-255 ~ 255)

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// ================= 함수 선언 =================
void handleEncoder();
void driveMotor(double out);
void parseSerialCommand();

void setup() {
    Serial.begin(115200); // todo - 여기는 왜 9600을 쓰지 않는지?
    
    pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT); pinMode(PIN_PWM, OUTPUT);
    pinMode(PIN_ENC_A, INPUT_PULLUP); pinMode(PIN_ENC_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), handleEncoder, RISING);

    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(-200, 200); 
    myPID.SetSampleTime(10);

    Serial.println("System Ready. Default Mode: PID_AUTO");
}

void loop() {
    // 1. 현재 각도 계산
    Input = (pulseCount / (PPR * GEAR_RATIO)) * 360.0;

    // 2. 시리얼 입력 처리
    parseSerialCommand(); // todo - 여기서 시리얼 입력이 무시되지는 않는지 (유니티에서 많이 겪어봄)

    // 3. 모드별 동작 수행
    if (currentMode == MANUAL_JOG) {
        // 수동 조그 모드: 시리얼로 입력된 manualSpeed로 직접 구동
        driveMotor(manualSpeed);
        Setpoint = Input; // 수동 이동 중에도 Setpoint를 현재치로 동기화 (모드 변경 시 튀지 않게)
    } 
    else {
        // PID 자동 제어 모드
        myPID.Compute();     
        driveMotor(Output);  
    }

    // 상태 모니터링 출력 (필요 시 주석 해제)
    // Serial.print("Mode:"); Serial.print(currentMode == PID_AUTO ? "AUTO" : "MANUAL");
    // Serial.print(" Set:"); Serial.print(Setpoint);
    // Serial.print(" In:"); Serial.println(Input);
}

void driveMotor(double out) {
    // [안전 장치] 리미트 도달 시 해당 방향 출력 차단
    if (Input >= MAX_LIMIT && out > 0) out = 0; 
    if (Input <= MIN_LIMIT && out < 0) out = 0;

    if (out > 0) {
        digitalWrite(PIN_IN1, HIGH); digitalWrite(PIN_IN2, LOW);
    } else if (out < 0) {
        digitalWrite(PIN_IN1, LOW); digitalWrite(PIN_IN2, HIGH);
    } else {
        digitalWrite(PIN_IN1, LOW); digitalWrite(PIN_IN2, LOW);
    }
    analogWrite(PIN_PWM, abs(out));
}

void parseSerialCommand() {
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        
        // 1. 모드 변경 명령어 (M)
        if (cmd == 'M' || cmd == 'm') {
            int modeVal = Serial.parseInt();
            if (modeVal == 0) {
                currentMode = PID_AUTO;
                manualSpeed = 0;
                Serial.println(">>> Mode Switched to: PID_AUTO");
            } else if (modeVal == 1) {
                currentMode = MANUAL_JOG;
                Serial.println(">>> Mode Switched to: MANUAL_JOG");
            }
            return;
        }

        // 2. 모드별 하부 명령어
        if (currentMode == PID_AUTO) {
            float val = Serial.parseFloat();
            switch (cmd) {
                case 'S': case 's': Setpoint = constrain(val, MIN_LIMIT, MAX_LIMIT); break;
                case 'P': case 'p': Kp = val; myPID.SetTunings(Kp, Ki, Kd); break;
                case 'I': case 'i': Ki = val; myPID.SetTunings(Kp, Ki, Kd); break;
                case 'D': case 'd': Kd = val; myPID.SetTunings(Kp, Ki, Kd); break;
            }
        } 
        else if (currentMode == MANUAL_JOG) {
            // 수동 조그 명령어
            switch (cmd) {
                case 'U': case 'u': manualSpeed = 150; Serial.println("Jog: UP"); break;   // 정방향 회전
                case 'D': case 'd': manualSpeed = -150; Serial.println("Jog: DOWN"); break; // 역방향 회전
                case 'X': case 'x': manualSpeed = 0; Serial.println("Jog: STOP"); break;    // 정지
            }
        }
    }
}

void handleEncoder() {
    if (digitalRead(PIN_ENC_B) == LOW) pulseCount++;
    else pulseCount--;
}