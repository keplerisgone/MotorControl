#include <Arduino.h>
#include <PID_v1.h>

// ================= 핀 정의 =================
#define PIN_JOY_X  A0
#define PIN_ENC_A  2
#define PIN_ENC_B  3
#define PIN_IN1    4
#define PIN_IN2    7
#define PIN_PWM    9

// ================= 시스템 상수 =================
const float GEAR_RATIO     = 27.0;
const float PPR            = 26.0;
const float PULSES_PER_REV = PPR * GEAR_RATIO;  // 출력축 1회전당 펄스

const float MAX_ANGLE      = 90.0;   // 소프트 리미트 (도)
const float MIN_ANGLE      = -90.0;

// ================= 조이스틱 파라미터 =================
const int   JOY_CENTER     = 512;
const int   JOY_DEADZONE   = 0; // 0이 아니면 작동 X
const int   JOY_MAX_RANGE  = 511;
const float JOY_ANGLE_RATE = 5;    // 루프당 최대 각도 변화량 (°)
                                      // 높을수록 조이스틱 반응 빠름

// ================= PID 설정 =================
// Setpoint: 목표 각도 (°)
// Input   : 현재 각도 (°, 엔코더로 계산)
// Output  : PWM (-255 ~ 255, 음수=역방향)
double Setpoint = 0;
double Input    = 0;
double Output   = 0;

double Kp = 3.0, Ki = 0.1, Kd = 0.5;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// ================= 엔코더 =================
volatile long pulseCount = 0;

// ================= 타이밍 =================
const unsigned long LOOP_MS = 20;
unsigned long prevTime = 0;

// ================= 함수 선언 =================
void  handleEncoder();
float measureAngle();
void  updateSetpoint(int rawX);
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

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255, 255);
  myPID.SetSampleTime(LOOP_MS);

  Serial.setTimeout(50);
  Serial.println("Position Control Ready.");
  Serial.println("Commands: P/I/D<value>");

  prevTime = millis();
}

// ===============================================
void loop() {
  unsigned long now = millis();
  if (now - prevTime < LOOP_MS) return;
  prevTime = now;

  // 1. 현재 각도 계산
  Input = measureAngle();

  // 2. 조이스틱 → Setpoint 각도 업데이트
  int rawX = analogRead(PIN_JOY_X);
  updateSetpoint(rawX);

  // 3. PID 계산 → 모터 출력
  // Deadzone 안이면 Setpoint = 현재 위치로 고정 (외력 저항)
  // motorStop() 호출 안 함 → PID가 계속 위치 유지
  myPID.Compute();
  driveMotor(Output);

  // 4. 시리얼
  parseSerial();
  printStatus();
}

// ===============================================
// 엔코더 → 현재 각도 (°)
// ===============================================
float measureAngle() {
  long snapshot;
  noInterrupts();
  snapshot = pulseCount;
  interrupts();

  return (snapshot / PULSES_PER_REV) * 360.0;
}

// ===============================================
// 조이스틱 → Setpoint 각도를 rate로 이동
// Deadzone 안: Setpoint 고정 (현재 위치 유지)
// Deadzone 밖: 조이스틱 비례로 Setpoint 이동
// ===============================================
void updateSetpoint(int rawX) {
  int offset = rawX - JOY_CENTER;

  if (abs(offset) <= JOY_DEADZONE) {
    // Deadzone → Setpoint 고정 (아무것도 안 함)
    // PID는 계속 현재 Setpoint를 유지하려 함 → 외력 저항
    return;
  }

  // 조이스틱 비례로 Setpoint 이동
  float norm = 0;
  if (offset > 0) {
    norm = (float)(offset - JOY_DEADZONE) / (JOY_MAX_RANGE - JOY_DEADZONE);
  } else {
    norm = (float)(offset + JOY_DEADZONE) / (JOY_MAX_RANGE - JOY_DEADZONE);
  }
  norm = constrain(norm, -1.0, 1.0);

  // S-curve 적용 (부호 유지)
  float sign   = (norm > 0) ? 1.0 : -1.0;
  float curved = sign * (abs(norm) * abs(norm) * (3.0 - 2.0 * abs(norm)));

  Setpoint += curved * JOY_ANGLE_RATE;
  Setpoint  = constrain(Setpoint, MIN_ANGLE, MAX_ANGLE);
}

// ===============================================
// 모터 출력
// ===============================================
void driveMotor(double out) {
  // 소프트 리미트: 한계 도달 시 해당 방향 차단
  if (Input >= MAX_ANGLE && out > 0) out = 0;
  if (Input <= MIN_ANGLE && out < 0) out = 0;

  int pwm = (int)constrain(abs(out), 0, 255);

  if (out > 1.0) {
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
    analogWrite(PIN_PWM, pwm);
  } else if (out < -1.0) {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
    analogWrite(PIN_PWM, pwm);
  } else {
    // Output이 거의 0 → 브레이크
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
    analogWrite(PIN_PWM, 0);
  }
}

void motorStop() {
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  analogWrite(PIN_PWM, 0);
}

// ===============================================
// 시리얼 명령어: P3.0 / I0.1 / D0.5
// ===============================================
void parseSerial() {
  if (!Serial.available()) return;

  char  cmd = Serial.read();
  float val = Serial.parseFloat();

  switch (cmd) {
    case 'P': case 'p':
      Kp = val; myPID.SetTunings(Kp, Ki, Kd);
      break;
    case 'I': case 'i':
      Ki = val; myPID.SetTunings(Kp, Ki, Kd);
      break;
    case 'D': case 'd':
      Kd = val; myPID.SetTunings(Kp, Ki, Kd);
      break;
  }
}

// 시리얼 플로터용
void printStatus() {
  Serial.print("Setpoint:"); Serial.print(Setpoint);
  Serial.print(" Input:");   Serial.print(Input);
  Serial.print(" Output:");  Serial.print(Output);
  Serial.print(" Kp:"); Serial.print(Kp);
  Serial.print(" Ki:"); Serial.print(Ki);
  Serial.print(" Kd:"); Serial.println(Kd);
}

// ===============================================
// 엔코더 ISR
// ===============================================
void handleEncoder() {
  if (digitalRead(PIN_ENC_B) == LOW) pulseCount++;
  else                               pulseCount--;
}