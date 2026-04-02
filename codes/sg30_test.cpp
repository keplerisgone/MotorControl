#include <Arduino.h>
#include <Servo.h>

// ================= 핀 정의 =================
#define PIN_JOY_SW  4    // 조이스틱 스위치
#define PIN_SERVO   6    // 서보 신호선 (PWM 핀)

// ================= 파라미터 =================
const int SERVO_CENTER  = 90;   // 초기 중립 각도
const int SERVO_STEP    = 5;    // 버튼 한 번에 움직이는 각도 (°)
const int SERVO_MIN     = 0;    // SG90 최소 각도
const int SERVO_MAX     = 180;  // SG90 최대 각도
const unsigned long DEBOUNCE_MS = 200;  // 디바운스 (ms)

// ================= 전역 변수 =================
Servo servo;
int   servoAngle = SERVO_CENTER;
bool  lastBtnState   = HIGH;
unsigned long lastPressTime = 0;

void setup() {
pinMode(PIN_JOY_SW, INPUT_PULLUP);  // 눌리면 LOW
servo.attach(PIN_SERVO);
servo.write(servoAngle);

Serial.begin(9600);
Serial.print("Servo init at "); Serial.print(servoAngle); Serial.println("°");
}

void loop() {
bool btnState = digitalRead(PIN_JOY_SW);
unsigned long now = millis();

// 버튼 눌림 감지 (HIGH→LOW 엣지) + 디바운스
if (btnState == LOW && lastBtnState == HIGH
  && (now - lastPressTime) > DEBOUNCE_MS) {

lastPressTime = now;
servoAngle = constrain(servoAngle + SERVO_STEP, SERVO_MIN, SERVO_MAX);
servo.write(servoAngle);

Serial.print("Servo → "); Serial.print(servoAngle); Serial.println("°");
}

lastBtnState = btnState;
}