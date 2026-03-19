#include <Arduino.h>

const int PIN_JOY_X = A0;

const int   JOY_CENTER    = 512;   // 조이스틱 중립값 (캘리브레이션으로 보정 가능)
const int   JOY_DEADZONE  = 40;    // ±이 범위 내는 정지로 처리
const int   JOY_MAX_RANGE = 470;   // 유효 범위 (512 - deadzone_edge)

float lastJoy;

float sCurveTransform(float t);

void setup() {
  Serial.begin(9600);
  Serial.println("System Ready. Move joystick to test.");
  delay(500);
}

void loop() {
  // check joystick 
  int rawX = analogRead(PIN_JOY_X);

  int offset = rawX - JOY_CENTER;
  float normalized;

  if (abs(offset) > JOY_DEADZONE) {
    if (offset > 0) {
      normalized = (float)(offset - JOY_DEADZONE) / (JOY_MAX_RANGE - JOY_DEADZONE);
    } else {
      normalized = (float)(-offset - JOY_DEADZONE) / (JOY_MAX_RANGE - JOY_DEADZONE);
    }

    normalized = constrain(normalized, 0.0, 1.0);
    float sCurve = sCurveTransform(normalized);
  } else {
    normalized = 0.0;
  }
  
  // Print Result (Debug)
  if (lastJoy != rawX) {
    Serial.print("RawX: "); Serial.print(rawX);
    Serial.print(" | offset: ");  Serial.print(offset);
    Serial.print(" | normalized: "); Serial.println(normalized);
  }
  
  lastJoy = rawX;

  delay(20);
}

float sCurveTransform(float t) {
  // Smoothstep 함수 (게임 엔진에서도 자주 사용)
  return t * t * (3.0 - 2.0 * t);
}