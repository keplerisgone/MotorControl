#include <Arduino.h>
#include <PID_v1.h>

// ================= 핀 정의 =================
#define PIN_JOY_X  A0
#define PIN_ENC_A  2       // 인터럽트 핀 (INT0)
#define PIN_ENC_B  3
#define PIN_IN1    4
#define PIN_IN2    7
#define PIN_PWM    9

// ================= 시스템 상수 =================
const float GEAR_RATIO    = 27.0;   // 기어비 (모터 스펙 확인)
const float PPR           = 26.0;   // 모터축 엔코더 펄스/회전
const float PULSES_PER_REV = PPR * GEAR_RATIO;  // 출력축 기준
const float MAX_RPM       = 60.0;   // 조이스틱 최대일 때 목표 RPM

// ================= 조이스틱 파라미터 =================
const int JOY_CENTER    = 512;
const int JOY_DEADZONE  = 0;
const int JOY_MAX_RANGE = 511;

// ================= PID 설정 =================
double Setpoint = 0;  // 목표 RPM  (-MAX_RPM ~ +MAX_RPM)
double Input    = 0;  // 현재 RPM  (엔코더로 계산)
double Output   = 0;  // PWM 출력  (-255 ~ 255, 음수=역방향)

double Kp = 3.0, Ki = 0.5, Kd = 0.05;  // 초기값 - 튜닝 필요

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// ================= 엔코더 변수 =================
volatile long pulseCount = 0;  // ISR에서 누적

// ================= 타이밍 =================
const unsigned long LOOP_MS = 20;   // 제어 루프 주기 (50Hz)
unsigned long prevTime = 0;

// ================= 함수 선언 =================
void  handleEncoder();
float measureRPM(unsigned long dt_ms);
float joystickToRPM(int rawX);
float sCurve(float t);
void  driveMotor(double out);
void  motorStop();
void  parseSerial();
void  printStatus();

// ===============================================
void setup() {
  Serial.begin(115200);

  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_PWM, OUTPUT);
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);

  motorStop();

  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), handleEncoder, RISING);

  // PID 초기화
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255, 255);
  myPID.SetSampleTime(LOOP_MS);

  Serial.setTimeout(50);  // parseFloat 블로킹 최소화
  Serial.println("Ready. Commands: P/I/D<value>, R<rpm>");
  prevTime = millis();
}

// ===============================================
void loop() {
  unsigned long now = millis();
  unsigned long dt  = now - prevTime;
  if (dt < LOOP_MS) return;
  prevTime = now;

  // 1. 현재 RPM 계산 (엔코더 스냅샷)
  Input = measureRPM(dt);

  // 2. 조이스틱 → 목표 RPM
  int rawX = analogRead(PIN_JOY_X);
  Setpoint = joystickToRPM(rawX);

  // 3. Deadzone이면 즉시 정지 (PID 상태 초기화)
  if (abs(Setpoint) < 0.5) {
    motorStop();
    myPID.SetMode(MANUAL);   // PID 적분 누적 초기화
    Output = 0;
    myPID.SetMode(AUTOMATIC);
    printStatus();
    return;
  }

  // 4. PID 계산 → 모터 출력
  myPID.Compute();
  driveMotor(Output);

  // 5. 시리얼 명령 처리 & 상태 출력
  parseSerial();
  printStatus();
}

// ===============================================
// 엔코더 스냅샷으로 RPM 계산
// dt_ms: 마지막 루프로부터 경과 시간 (ms)
// ===============================================
float measureRPM(unsigned long dt_ms) {
  long snapshot;
  noInterrupts();
  snapshot    = pulseCount;
  pulseCount  = 0;          // 다음 루프를 위해 초기화
  interrupts();

  // snapshot 펄스 / 출력축 1회전 펄스 * (60000ms / dt_ms) = RPM
  float rpm = (snapshot / PULSES_PER_REV) * (60000.0 / dt_ms);
  return rpm;
}

// ===============================================
// 조이스틱 rawX → 목표 RPM (S-curve 적용)
// ===============================================
float joystickToRPM(int rawX) {
  int offset = rawX - JOY_CENTER;

  if (abs(offset) < JOY_DEADZONE) return 0.0;

  float norm;
  int   sign;
  if (offset > 0) {
    norm = (float)(offset - JOY_DEADZONE) / (JOY_MAX_RANGE - JOY_DEADZONE);
    sign = 1;
  } else {
    norm = (float)(-offset - JOY_DEADZONE) / (JOY_MAX_RANGE - JOY_DEADZONE);
    sign = -1;
  }

  norm = constrain(norm, 0.0, 1.0);
  return sign * sCurve(norm) * MAX_RPM;
}

// S-curve (smoothstep)
float sCurve(float t) {
  return t * t * (3.0 - 2.0 * t);
}

// ===============================================
// 모터 출력 (Output 음수=역방향, 양수=정방향)
// ===============================================
void driveMotor(double out) {
  int pwm = (int)constrain(abs(out), 0, 255);

  if (out > 0.5) {
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
  } else if (out < -0.5) {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
  } else {
    motorStop();
    return;
  }
  analogWrite(PIN_PWM, pwm);
}

void motorStop() {
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  analogWrite(PIN_PWM, 0);
}

// ===============================================
// 시리얼 명령어
//   P3.0  → Kp = 3.0
//   I0.5  → Ki = 0.5
//   D0.05 → Kd = 0.05
//   R60   → MAX_RPM 실시간 변경 (별도 변수 필요 시)
// ===============================================
void parseSerial() {
  if (!Serial.available()) return;

  char  cmd = Serial.read();
  float val = Serial.parseFloat();

  switch (cmd) {
    case 'P': case 'p':
      Kp = val; myPID.SetTunings(Kp, Ki, Kd);
      Serial.print("Kp = "); Serial.println(Kp);
      break;
    case 'I': case 'i':
      Ki = val; myPID.SetTunings(Kp, Ki, Kd);
      Serial.print("Ki = "); Serial.println(Ki);
      break;
    case 'D': case 'd':
      Kd = val; myPID.SetTunings(Kp, Ki, Kd);
      Serial.print("Kd = "); Serial.println(Kd);
      break;
  }
}

// 시리얼 플로터용 출력
void printStatus() {
  Serial.print("Setpoint:"); Serial.print(Setpoint);
  Serial.print(" Input:");   Serial.print(Input);
  Serial.print(" Output:");  Serial.println(Output);
}

// ===============================================
// 엔코더 ISR
// A상 RISING 시 B상으로 방향 판별
// ===============================================
void handleEncoder() {
  if (digitalRead(PIN_ENC_B) == LOW) pulseCount++;
  else                               pulseCount--;
}
