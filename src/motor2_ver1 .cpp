#include <Arduino.h>
// ============================================================
// Motor Smooth Control via PS2 Joystick (S-curve + Ramping)
// Board   : Arduino Uno
// Driver  : SZH-GNP521
// Motor   : IG-30GM 03TYPE (12V)
// Joystick: HW-504
// ============================================================

// --- 핀 정의 ---
const int PIN_JOY_X = A0;   // 조이스틱 X축
const int PIN_JOY_Y = A1;   // 조이스틱 X축
const int PIN_IN1   = 4;    // 모터 방향 A1
const int PIN_IN2   = 7;    // 모터 방향 A2
const int PIN_PWM   = 9;    // 모터 속도 (PWM)

// motor 2
const int PIN_IN1_M2 = 5;
const int PIN_IN2_M2 = 6;
const int PIN_PWM_M2 = 10;

// --- 파라미터 ---
const int   JOY_CENTER    = 512;   // 조이스틱 중립값 (캘리브레이션으로 보정 가능)
const int   JOY_DEADZONE  = 40;    // ±이 범위 내는 정지로 처리
const int   JOY_MAX_RANGE = 470;   // 유효 범위 (512 - deadzone_edge)
const float RAMP_RATE     = 24.0;   // 루프당 최대 PWM 변화량 (낮을수록 부드러움)
const int   PWM_MIN       = 0;     // 최소 PWM
const int   PWM_ABS_MAX   = 255;   // 절대 상한 (이 이상은 못 올림)
const int   PWM_ABS_MIN   = 10;    // 절대 하한 (이 이하는 못 내림)

// --- 모터별 PWM 최대치 (런타임 조절 가능) ---
int pwmMax_M1 = 150;
int pwmMax_M2 = 150;

float sCurveTransform(float t);
void  motorDrive(int pinIN1, int pinIN2, int pinPWM, int dir, int pwm);
void  motorStop(int pinIN1, int pinIN2, int pinPWM);
void  updateMotor(int rawVal, float &currentPWM, int pinIN1, int pinIN2, int pinPWM, int pwmMax);
void  handleSerial();
void  printPWMMax();

// --- 상태 변수 ---
float currentPWM = 0.0;   // 현재 출력 PWM (float으로 부드러운 ramping)
float currentPWM_M2 = 0.0; 

void setup() {
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_PWM, OUTPUT);
  pinMode(PIN_IN1_M2, OUTPUT);
  pinMode(PIN_IN2_M2, OUTPUT);
  pinMode(PIN_PWM_M2, OUTPUT);

  // 초기 안전 상태: 모터 정지
  motorStop(PIN_IN1, PIN_IN2, PIN_PWM);
  motorStop(PIN_IN1_M2, PIN_IN2_M2, PIN_PWM_M2);

  Serial.begin(9600);
  Serial.println("System Ready. Move joystick to control motor.");
  Serial.println("q/a: M1 PWM +/-10  |  w/s: M2 PWM +/-10  |  p: 현재값 출력");
  printPWMMax();
  delay(500);  // 시스템 안정화 대기
}

void loop() {
  handleSerial();

  // 1. 조이스틱 읽기
  int rawX = analogRead(PIN_JOY_X);
  int rawY = analogRead(PIN_JOY_Y);

  updateMotor(rawX, currentPWM, PIN_IN1, PIN_IN2, PIN_PWM, pwmMax_M1);
  updateMotor(rawY, currentPWM_M2, PIN_IN1_M2, PIN_IN2_M2, PIN_PWM_M2, pwmMax_M2);

  // 디버그 출력 (시리얼 모니터)
  Serial.print("RawX: "); Serial.print(rawX);
  Serial.print(" | M1_PWM: "); Serial.print((int)currentPWM);
  Serial.print(" | RawY: "); Serial.print(rawY);
  Serial.print(" | M2_PWM: "); Serial.println((int)currentPWM_M2);

  delay(20);  // 50Hz 제어 루프
}

void handleSerial() {
  if (!Serial.available()) return;

  char cmd = Serial.read();

  switch (cmd) {
    case 'q':
      pwmMax_M1 = constrain(pwmMax_M1 + 10, PWM_ABS_MIN, PWM_ABS_MAX);
      Serial.print("[M1] PWM_MAX +10 → "); Serial.println(pwmMax_M1);
      break;
    case 'a':
      pwmMax_M1 = constrain(pwmMax_M1 - 10, PWM_ABS_MIN, PWM_ABS_MAX);
      Serial.print("[M1] PWM_MAX -10 → "); Serial.println(pwmMax_M1);
      break;
    case 'w':
      pwmMax_M2 = constrain(pwmMax_M2 + 10, PWM_ABS_MIN, PWM_ABS_MAX);
      Serial.print("[M2] PWM_MAX +10 → "); Serial.println(pwmMax_M2);
      break;
    case 's':
      pwmMax_M2 = constrain(pwmMax_M2 - 10, PWM_ABS_MIN, PWM_ABS_MAX);
      Serial.print("[M2] PWM_MAX -10 → "); Serial.println(pwmMax_M2);
      break;
    case 'p':
      printPWMMax();
      break;
    default:
      break;  // 그 외 입력은 무시
  }
}

void printPWMMax() {
  Serial.print("[현재] M1 PWM_MAX: "); Serial.print(pwmMax_M1);
  Serial.print("  |  M2 PWM_MAX: "); Serial.println(pwmMax_M2);
}

void updateMotor(int rawVal, float &currentPWM, int pinIN1, int pinIN2, int pinPWM, int pwmMax) {
  int offset = rawVal - JOY_CENTER;

  float targetPWM = 0.0;
  int   direction = 0;  // 0=정지, 1=정방향, -1=역방향

  if (abs(offset) > JOY_DEADZONE) {
    // deadzone 제거 후 유효 범위로 정규화
    float normalized;
    if (offset > 0) {
      normalized = (float)(offset - JOY_DEADZONE) / (JOY_MAX_RANGE - JOY_DEADZONE);
      direction = 1;
    } else {
      normalized = (float)(-offset - JOY_DEADZONE) / (JOY_MAX_RANGE - JOY_DEADZONE);
      direction = -1;
    }

    // 범위 클램핑 (0.0 ~ 1.0)
    normalized = constrain(normalized, 0.0, 1.0);
    targetPWM = sCurveTransform(normalized) * pwmMax;  // S-curve 적용 후 최대 PWM 곱하기

  } else {
    // Deadzone 내 → 정지
    motorStop(pinIN1, pinIN2, pinPWM);
    currentPWM = 0.0;
  }

  // 6. Ramping: 현재 PWM을 목표 PWM 쪽으로 서서히 이동
  if (currentPWM < targetPWM) {
    currentPWM = min(currentPWM + RAMP_RATE, targetPWM);
  } else if (currentPWM > targetPWM) {
    currentPWM = max(currentPWM - RAMP_RATE, targetPWM);
  }

  // 7. 모터 출력
  int pwmOut = (int)constrain(currentPWM, PWM_MIN, pwmMax);
  motorDrive(pinIN1, pinIN2, pinPWM, direction, pwmOut);
}

// ============================================================
// S-curve 변환 함수
// ============================================================
float sCurveTransform(float t) {
  // Smoothstep 함수 (게임 엔진에서도 자주 사용)
  return t * t * (3.0 - 2.0 * t);
}

// ============================================================
// 모터 드라이브 함수
// dir: 1=정방향, -1=역방향, 0=정지
// ============================================================
void motorDrive(int pinIN1, int pinIN2, int pinPWM, int dir, int pwm) {
  if (dir == 1) {
    digitalWrite(pinIN1, HIGH);
    digitalWrite(pinIN2, LOW);
  } else if (dir == -1) {
    digitalWrite(pinIN1, LOW);
    digitalWrite(pinIN2, HIGH);
  } else {
    motorStop(pinIN1, pinIN2, pinPWM);
    return;
  }
  analogWrite(pinPWM, pwm);
}

void motorStop(int pinIN1, int pinIN2, int pinPWM) {
  digitalWrite(pinIN1, LOW);
  digitalWrite(pinIN2, LOW);
  analogWrite(pinPWM, 0);
}
