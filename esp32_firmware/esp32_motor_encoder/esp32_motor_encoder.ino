/*
 * ╔══════════════════════════════════════════════════════════════════════╗
 * ║  SMALLBOT — ESP32 Firmware  (v3 — MOTOR SELF-TEST + DEBOUNCE)      ║
 * ╠══════════════════════════════════════════════════════════════════════╣
 * ║  • ISR debouncing for noisy encoder pins 34/35                      ║
 * ║  • Startup motor self-test (brief spin to verify hardware)          ║
 * ║  • Serial echo of received commands for debugging                   ║
 * ╚══════════════════════════════════════════════════════════════════════╝
 */

#include <Arduino.h>

// ═══════════════════════════════════════════════════════════════════════
//  PIN DEFINITIONS
// ═══════════════════════════════════════════════════════════════════════
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

// ═══════════════════════════════════════════════════════════════════════
//  HARDWARE CONSTANTS
// ═══════════════════════════════════════════════════════════════════════
const float TPR_L       = 349.0f;
const float TPR_R       = 362.0f;
const float WHEEL_DIA   = 0.043f;  
const float WHEEL_BASE  = 0.140f;  
const float WHEEL_CIRC  = PI * WHEEL_DIA; 

const int   PWM_MIN     = 55;      
const int   PWM_MAX     = 255;     

// ═══════════════════════════════════════════════════════════════════════
//  ENCODER STATE  (with ISR debouncing)
// ═══════════════════════════════════════════════════════════════════════
volatile int32_t enc_left  = 0;
volatile int32_t enc_right = 0;

#define ENC_DEBOUNCE_US  200

volatile uint32_t last_isr_l = 0;
volatile uint32_t last_isr_r = 0;

void IRAM_ATTR isrLeft() {
  uint32_t now = micros();
  if ((now - last_isr_l) < ENC_DEBOUNCE_US) return;
  last_isr_l = now;
  if (digitalRead(ENC_L_B) == HIGH) enc_left++; else enc_left--;
}

void IRAM_ATTR isrRight() {
  uint32_t now = micros();
  if ((now - last_isr_r) < ENC_DEBOUNCE_US) return;
  last_isr_r = now;
  if (digitalRead(ENC_R_B) == HIGH) enc_right--; else enc_right++;
}

// ═══════════════════════════════════════════════════════════════════════
//  PID CONTROLLER
// ═══════════════════════════════════════════════════════════════════════
const float Kp = 1.5f; 
const float Ki = 0.2f; 

float error_sum_L = 0;
float error_sum_R = 0;

float target_speed_L = 0;
float target_speed_R = 0;

// ═══════════════════════════════════════════════════════════════════════
//  MOTOR DRIVER
// ═══════════════════════════════════════════════════════════════════════

// PWM write helper — handles both old (channel) and new (pin) LEDC API
void writePWM_L(int pwm) {
  #if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(L_ENA, pwm);
  #else
    ledcWrite(0, pwm);
  #endif
}

void writePWM_R(int pwm) {
  #if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(R_ENB, pwm);
  #else
    ledcWrite(1, pwm);
  #endif
}

void motorInit() {
  pinMode(L_IN1, OUTPUT); pinMode(L_IN2, OUTPUT);
  pinMode(R_IN3, OUTPUT); pinMode(R_IN4, OUTPUT);
  
  // Stop motors first
  digitalWrite(L_IN1, LOW); digitalWrite(L_IN2, LOW);
  digitalWrite(R_IN3, LOW); digitalWrite(R_IN4, LOW);
  
  #if ESP_ARDUINO_VERSION_MAJOR >= 3
    // ESP32 Arduino Core v3.x: pin-based API
    ledcAttach(L_ENA, 20000, 8);
    ledcAttach(R_ENB, 20000, 8);
  #else
    // ESP32 Arduino Core v2.x: channel-based API
    ledcSetup(0, 20000, 8);
    ledcSetup(1, 20000, 8);
    ledcAttachPin(L_ENA, 0);
    ledcAttachPin(R_ENB, 1);
  #endif
  
  writePWM_L(0);
  writePWM_R(0);
}

void stopAll() {
  digitalWrite(L_IN1, LOW); digitalWrite(L_IN2, LOW);
  digitalWrite(R_IN3, LOW); digitalWrite(R_IN4, LOW);
  writePWM_L(0);
  writePWM_R(0);
}

void applyPower(int in1, int in2, bool isLeft, int pwm) {
  if (pwm > 0) {
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  } else if (pwm < 0) {
    digitalWrite(in1, LOW);  digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);  digitalWrite(in2, LOW);
    if (isLeft) writePWM_L(0); else writePWM_R(0);
    return;
  }
  
  int final_pwm = abs(pwm);
  if (final_pwm < PWM_MIN) final_pwm = PWM_MIN;
  if (final_pwm > PWM_MAX) final_pwm = PWM_MAX;
  
  if (isLeft) writePWM_L(final_pwm);
  else        writePWM_R(final_pwm);
}

void setTargetVelocities(float v, float omega) {
  target_speed_L = v - omega * (WHEEL_BASE / 2.0f);
  target_speed_R = v + omega * (WHEEL_BASE / 2.0f);
}

// ═══════════════════════════════════════════════════════════════════════
//  SERIAL PARSING
// ═══════════════════════════════════════════════════════════════════════
uint32_t last_cmd_ms = 0;
const uint32_t CMD_TIMEOUT_MS = 500;   
String serial_buf = "";
bool got_first_cmd = false;

void parseSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      serial_buf.trim();
      if (serial_buf.length() >= 3 && serial_buf[0] == 'C' && serial_buf[1] == ':') {
        int comma = serial_buf.indexOf(',', 2);
        if (comma > 2) {
          float v     = serial_buf.substring(2, comma).toFloat();
          float omega = serial_buf.substring(comma + 1).toFloat();
          setTargetVelocities(v, omega);
          last_cmd_ms = millis();
          got_first_cmd = true;
          digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
          // Debug echo — prints "D:" lines (ignored by ROS bridge which only parses "E:")
          Serial.print("D:CMD v=");
          Serial.print(v, 4);
          Serial.print(" w=");
          Serial.print(omega, 4);
          Serial.print(" tL=");
          Serial.print(target_speed_L, 4);
          Serial.print(" tR=");
          Serial.println(target_speed_R, 4);
        }
      } else if (serial_buf.length() > 0 && serial_buf[0] == 'R') {
        noInterrupts(); enc_left = 0; enc_right = 0; interrupts();
        error_sum_L = 0; error_sum_R = 0; 
        Serial.println(">>> ENCODERS & PID RESET <<<");
      }
      serial_buf = "";
    } else if (c != '\r') {
      serial_buf += c;
      if (serial_buf.length() > 32) serial_buf = ""; 
    }
  }
}

// ═══════════════════════════════════════════════════════════════════════
//  MOTOR SELF-TEST  (runs once at startup to verify hardware)
// ═══════════════════════════════════════════════════════════════════════
void motorSelfTest() {
  Serial.println("D:SELF-TEST: Left motor forward...");
  digitalWrite(L_IN1, HIGH); digitalWrite(L_IN2, LOW);
  writePWM_L(100);
  delay(300);
  stopAll();
  delay(200);

  Serial.println("D:SELF-TEST: Right motor forward...");
  digitalWrite(R_IN3, HIGH); digitalWrite(R_IN4, LOW);
  writePWM_R(100);
  delay(300);
  stopAll();
  delay(200);

  Serial.println("D:SELF-TEST: Complete");
}

// ═══════════════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(100);
  
  pinMode(STATUS_LED, OUTPUT); digitalWrite(STATUS_LED, LOW);
  motorInit();

  pinMode(ENC_L_A, INPUT); 
  pinMode(ENC_L_B, INPUT);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrLeft,  RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrRight, RISING);

  for (int i = 0; i < 3; i++) {
    digitalWrite(STATUS_LED, HIGH); delay(150);
    digitalWrite(STATUS_LED, LOW);  delay(150);
  }
  
  Serial.println("SMALLBOT PID v3 READY");
  
  // Self-test: briefly spin each motor so user can verify hardware
  motorSelfTest();
  
  // Set timestamp to NOW so timeout doesn't fire before first command
  last_cmd_ms = millis();
}

// ═══════════════════════════════════════════════════════════════════════
//  MAIN CONTROL LOOP (20 Hz)
// ═══════════════════════════════════════════════════════════════════════
uint32_t last_loop_ms = 0;
const uint32_t LOOP_PERIOD_MS = 50; 

int32_t prev_ticks_L = 0;
int32_t prev_ticks_R = 0;

void loop() {
  parseSerial();

  // Command timeout — only after first command received
  if (got_first_cmd && (millis() - last_cmd_ms) > CMD_TIMEOUT_MS) {
    target_speed_L = 0.0f;
    target_speed_R = 0.0f;
  }

  uint32_t now = millis();
  if ((now - last_loop_ms) < LOOP_PERIOD_MS) return;
  
  float dt = (now - last_loop_ms) / 1000.0f;
  last_loop_ms = now;

  noInterrupts();
  int32_t ticks_L = enc_left;
  int32_t ticks_R = enc_right;
  interrupts();

  int32_t delta_L = ticks_L - prev_ticks_L;
  int32_t delta_R = ticks_R - prev_ticks_R;
  prev_ticks_L = ticks_L;
  prev_ticks_R = ticks_R;

  float current_speed_L = (delta_L / TPR_L) * WHEEL_CIRC / dt;
  float current_speed_R = (delta_R / TPR_R) * WHEEL_CIRC / dt;

  // PID for left motor
  if (abs(target_speed_L) < 0.001f) {
    applyPower(L_IN1, L_IN2, true, 0);
    error_sum_L = 0; 
  } else {
    float error_L = target_speed_L - current_speed_L;
    error_sum_L += error_L * dt;
    error_sum_L = constrain(error_sum_L, -50.0f, 50.0f);
    float pwm_L = (target_speed_L * 400.0f) + (Kp * error_L) + (Ki * error_sum_L);
    applyPower(L_IN1, L_IN2, true, (int)pwm_L);
  }

  // PID for right motor
  if (abs(target_speed_R) < 0.001f) {
    applyPower(R_IN3, R_IN4, false, 0);
    error_sum_R = 0; 
  } else {
    float error_R = target_speed_R - current_speed_R;
    error_sum_R += error_R * dt;
    error_sum_R = constrain(error_sum_R, -50.0f, 50.0f);
    float pwm_R = (target_speed_R * 400.0f) + (Kp * error_R) + (Ki * error_sum_R);
    applyPower(R_IN3, R_IN4, false, (int)pwm_R);
  }

  Serial.print("E:");
  Serial.print(ticks_L);
  Serial.print(',');
  Serial.println(ticks_R);
}
