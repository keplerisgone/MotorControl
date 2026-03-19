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
const int PIN_IN1   = 4;    // 모터 방향 A1
const int PIN_IN2   = 7;    // 모터 방향 A2
const int PIN_PWM   = 9;    // 모터 속도 (PWM)

// --- 파라미터 ---
const int   JOY_CENTER    = 512;   // 조이스틱 중립값 (캘리브레이션으로 보정 가능)
const int   JOY_DEADZONE  = 40;    // ±이 범위 내는 정지로 처리
const int   JOY_MAX_RANGE = 470;   // 유효 범위 (512 - deadzone_edge)
const float RAMP_RATE     = 8.0;   // 루프당 최대 PWM 변화량 (낮을수록 부드러움)
const int   PWM_MIN       = 0;     // 최소 PWM
const int   PWM_MAX       = 200;   // 최대 PWM (255 미만으로 제한 → 안전 마진)

float sCurveTransform(float t);
void motorDrive(int dir, int pwm);
void motorStop();

// --- 상태 변수 ---
float currentPWM = 0.0;   // 현재 출력 PWM (float으로 부드러운 ramping)

void setup() {
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_PWM, OUTPUT);

  // 초기 안전 상태: 모터 정지
  motorStop();

  Serial.begin(9600);
  Serial.println("System Ready. Move joystick to control motor.");
  delay(500);  // 시스템 안정화 대기
}

void loop() {
  // 1. 조이스틱 읽기
  int rawX = analogRead(PIN_JOY_X);

  // 2. 중립 기준으로 정규화 (-1.0 ~ +1.0)
  int offset = rawX - JOY_CENTER;

  float targetPWM = 0.0;
  int   direction = 0;  // 0=정지, 1=정방향, -1=역방향

  // 3. Deadzone 처리
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

    // 4. S-curve 변환 (3차 함수: 느리게 시작 → 빠르게 → 느리게)
    float sCurve = sCurveTransform(normalized);

    // 5. PWM 목표값 계산
    targetPWM = sCurve * PWM_MAX;

  } else {
    // Deadzone 내 → 정지
    motorStop();
  }

  // 6. Ramping: 현재 PWM을 목표 PWM 쪽으로 서서히 이동
  if (currentPWM < targetPWM) {
    currentPWM = min(currentPWM + RAMP_RATE, targetPWM);
  } else if (currentPWM > targetPWM) {
    currentPWM = max(currentPWM - RAMP_RATE, targetPWM);
  }

  // 7. 모터 출력
  int pwmOut = (int)constrain(currentPWM, PWM_MIN, PWM_MAX);
  motorDrive(direction, pwmOut);

  // 디버그 출력 (시리얼 모니터)
  Serial.print("RawX: "); Serial.print(rawX);
  Serial.print(" | Dir: ");  Serial.print(direction);
  Serial.print(" | Target: "); Serial.print((int)targetPWM);
  Serial.print(" | PWM: "); Serial.println(pwmOut);

  delay(20);  // 50Hz 제어 루프
}

// ============================================================
// S-curve 변환 함수
// 입력: 0.0 ~ 1.0 (정규화된 조이스틱 값)
// 출력: 0.0 ~ 1.0 (S-curve 적용값)
// 수식: smoothstep = 3t² - 2t³  (S자 모양, 시작/끝 부드러움)
// ============================================================
float sCurveTransform(float t) {
  // Smoothstep 함수 (게임 엔진에서도 자주 사용)
  return t * t * (3.0 - 2.0 * t);

  // 대안 1 - 3차 함수 (중간 더 민감):
  // return t * t * t;

  // 대안 2 - 5차 smoothstep (더 부드러운 경계):
  // return t * t * t * (t * (t * 6.0 - 15.0) + 10.0);
}

// ============================================================
// 모터 드라이브 함수
// dir: 1=정방향, -1=역방향, 0=정지
// ============================================================
void motorDrive(int dir, int pwm) {
  if (dir == 1) {
    // 정방향: IN1=HIGH, IN2=LOW (드라이버 스펙 참조)
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
    analogWrite(PIN_PWM, pwm);
  } else if (dir == -1) {
    // 역방향: IN1=LOW, IN2=HIGH
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
    analogWrite(PIN_PWM, pwm);
  } else {
    motorStop();
  }
}

void motorStop() {
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  analogWrite(PIN_PWM, 0);
}
