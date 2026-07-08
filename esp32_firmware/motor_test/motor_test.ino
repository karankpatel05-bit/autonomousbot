/*
 * ╔══════════════════════════════════════════════════════════════════════╗
 * ║  MOTOR TEST — Simple N20 encoder motor hardware test                ║
 * ║                                                                      ║
 * ║  Upload this to ESP32. Open Serial Monitor at 115200 baud.          ║
 * ║  It will:                                                            ║
 * ║    1. Spin LEFT motor forward 2s, then backward 2s                  ║
 * ║    2. Spin RIGHT motor forward 2s, then backward 2s                 ║
 * ║    3. Spin BOTH motors forward 2s                                   ║
 * ║    4. Print encoder ticks continuously                              ║
 * ║                                                                      ║
 * ║  If motors don't spin → wiring or battery problem                   ║
 * ║  If encoders don't count → encoder wiring problem                   ║
 * ╚══════════════════════════════════════════════════════════════════════╝
 */

#include <Arduino.h>

// ── Pin Definitions (same as main firmware) ──
#define L_IN1      26
#define L_IN2      27
#define L_ENA      25

#define R_IN3      32
#define R_IN4      33
#define R_ENB       4

#define ENC_L_A    34
#define ENC_L_B    35
#define ENC_R_A    18
#define ENC_R_B    19
#define STATUS_LED  2

// ── Encoder counters ──
volatile int32_t enc_left  = 0;
volatile int32_t enc_right = 0;

void IRAM_ATTR isrLeft()  { enc_left++;  }
void IRAM_ATTR isrRight() { enc_right++; }

// ── Motor helpers ──
void stopAll() {
  digitalWrite(L_IN1, LOW); digitalWrite(L_IN2, LOW);
  digitalWrite(R_IN3, LOW); digitalWrite(R_IN4, LOW);
  #if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(L_ENA, 0);
    ledcWrite(R_ENB, 0);
  #else
    ledcWrite(0, 0);
    ledcWrite(1, 0);
  #endif
}

void setLeftMotor(int pwm) {
  if (pwm > 0) {
    digitalWrite(L_IN1, HIGH); digitalWrite(L_IN2, LOW);
  } else if (pwm < 0) {
    digitalWrite(L_IN1, LOW); digitalWrite(L_IN2, HIGH);
  } else {
    digitalWrite(L_IN1, LOW); digitalWrite(L_IN2, LOW);
  }
  #if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(L_ENA, abs(pwm));
  #else
    ledcWrite(0, abs(pwm));
  #endif
}

void setRightMotor(int pwm) {
  if (pwm > 0) {
    digitalWrite(R_IN3, HIGH); digitalWrite(R_IN4, LOW);
  } else if (pwm < 0) {
    digitalWrite(R_IN3, LOW); digitalWrite(R_IN4, HIGH);
  } else {
    digitalWrite(R_IN3, LOW); digitalWrite(R_IN4, LOW);
  }
  #if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(R_ENB, abs(pwm));
  #else
    ledcWrite(1, abs(pwm));
  #endif
}

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(STATUS_LED, OUTPUT);
  pinMode(L_IN1, OUTPUT); pinMode(L_IN2, OUTPUT);
  pinMode(R_IN3, OUTPUT); pinMode(R_IN4, OUTPUT);

  // PWM setup
  #if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(L_ENA, 20000, 8);
    ledcAttach(R_ENB, 20000, 8);
  #else
    ledcSetup(0, 20000, 8);
    ledcSetup(1, 20000, 8);
    ledcAttachPin(L_ENA, 0);
    ledcAttachPin(R_ENB, 1);
  #endif

  stopAll();

  // Encoder pins
  pinMode(ENC_L_A, INPUT);
  pinMode(ENC_L_B, INPUT);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrLeft,  RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrRight, RISING);

  Serial.println("========================================");
  Serial.println("  SMALLBOT MOTOR + ENCODER TEST");
  Serial.println("========================================");
  Serial.println();

  // ── TEST 1: Left motor forward ──
  enc_left = 0; enc_right = 0;
  Serial.println("[TEST 1] LEFT motor FORWARD at PWM 120...");
  setLeftMotor(120);
  delay(2000);
  stopAll();
  Serial.print("  Encoder L="); Serial.print(enc_left);
  Serial.print("  R="); Serial.println(enc_right);
  Serial.println(enc_left > 10 ? "  >> LEFT ENCODER: OK" : "  >> LEFT ENCODER: FAIL (no ticks)");
  delay(500);

  // ── TEST 2: Left motor backward ──
  enc_left = 0; enc_right = 0;
  Serial.println("[TEST 2] LEFT motor BACKWARD at PWM 120...");
  setLeftMotor(-120);
  delay(2000);
  stopAll();
  Serial.print("  Encoder L="); Serial.print(enc_left);
  Serial.print("  R="); Serial.println(enc_right);
  delay(500);

  // ── TEST 3: Right motor forward ──
  enc_left = 0; enc_right = 0;
  Serial.println("[TEST 3] RIGHT motor FORWARD at PWM 120...");
  setRightMotor(120);
  delay(2000);
  stopAll();
  Serial.print("  Encoder L="); Serial.print(enc_left);
  Serial.print("  R="); Serial.println(enc_right);
  Serial.println(enc_right > 10 ? "  >> RIGHT ENCODER: OK" : "  >> RIGHT ENCODER: FAIL (no ticks)");
  delay(500);

  // ── TEST 4: Right motor backward ──
  enc_left = 0; enc_right = 0;
  Serial.println("[TEST 4] RIGHT motor BACKWARD at PWM 120...");
  setRightMotor(-120);
  delay(2000);
  stopAll();
  Serial.print("  Encoder L="); Serial.print(enc_left);
  Serial.print("  R="); Serial.println(enc_right);
  delay(500);

  // ── TEST 5: Both motors forward ──
  enc_left = 0; enc_right = 0;
  Serial.println("[TEST 5] BOTH motors FORWARD at PWM 120...");
  setLeftMotor(120);
  setRightMotor(120);
  delay(2000);
  stopAll();
  Serial.print("  Encoder L="); Serial.print(enc_left);
  Serial.print("  R="); Serial.println(enc_right);
  delay(500);

  Serial.println();
  Serial.println("========================================");
  Serial.println("  TESTS COMPLETE — live encoder below");
  Serial.println("========================================");
  Serial.println();

  enc_left = 0; enc_right = 0;
}

void loop() {
  // Print live encoder values every 500ms — spin wheels by hand to check
  static uint32_t last_print = 0;
  if (millis() - last_print >= 500) {
    last_print = millis();
    Serial.print("Live encoders:  L=");
    Serial.print(enc_left);
    Serial.print("  R=");
    Serial.println(enc_right);
    digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
  }
}
