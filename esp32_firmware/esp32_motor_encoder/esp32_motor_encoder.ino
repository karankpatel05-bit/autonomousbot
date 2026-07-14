/*
 * ╔══════════════════════════════════════════════════════════════════════╗
 * ║  SMALLBOT — ESP32 Firmware  (PID + DEBOUNCED ENCODERS)              ║
 * ╠══════════════════════════════════════════════════════════════════════╣
 * ║  RISING edge interrupts with 200µs debounce for noisy pins 34/35   ║
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
//  LEDC (hardware PWM)
// ═══════════════════════════════════════════════════════════════════════
#define LEDC_FREQ       20000   
#define LEDC_RES        8       
#define LEDC_CH_L       0       
#define LEDC_CH_R       1       

// ═══════════════════════════════════════════════════════════════════════
//  HARDWARE CONSTANTS
// ═══════════════════════════════════════════════════════════════════════
const float TPR_L       = 349.0f;
const float TPR_R       = 362.0f;
const float WHEEL_DIA   = 0.043f;  
const float WHEEL_BASE  = 0.400f;  // wheel-centre to wheel-centre (400mm body width)

const float WHEEL_CIRC  = PI * WHEEL_DIA; 

const int   PWM_MIN     = 55;      
const int   PWM_MAX     = 255;     

// ═══════════════════════════════════════════════════════════════════════
//  ENCODER STATE  (with ISR debouncing for noisy pins 34/35)
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
  if (digitalRead(ENC_L_B) == HIGH) enc_left--; else enc_left++;
}

void IRAM_ATTR isrRight() {
  uint32_t now = micros();
  if ((now - last_isr_r) < ENC_DEBOUNCE_US) return;
  last_isr_r = now;
  if (digitalRead(ENC_R_B) == HIGH) enc_right++; else enc_right--;
}

// ═══════════════════════════════════════════════════════════════════════
//  PID CONTROLLER VARIABLES
// ═══════════════════════════════════════════════════════════════════════
const float Kp = 1.5f; 
const float Ki = 0.2f; 

float error_sum_L = 0;
float error_sum_R = 0;

float target_speed_L = 0; // m/s
float target_speed_R = 0; // m/s

// ═══════════════════════════════════════════════════════════════════════
//  MOTOR DRIVER INTERFACE
// ═══════════════════════════════════════════════════════════════════════
void motorInit() {
  pinMode(L_IN1, OUTPUT); pinMode(L_IN2, OUTPUT);
  pinMode(R_IN3, OUTPUT); pinMode(R_IN4, OUTPUT);
  
  ledcSetup(LEDC_CH_L, LEDC_FREQ, LEDC_RES);
  ledcSetup(LEDC_CH_R, LEDC_FREQ, LEDC_RES);
  ledcAttachPin(L_ENA, LEDC_CH_L);
  ledcAttachPin(R_ENB, LEDC_CH_R);
  
  ledcWrite(LEDC_CH_L, 0); 
  ledcWrite(LEDC_CH_R, 0);
  
  digitalWrite(L_IN1, LOW); digitalWrite(L_IN2, LOW);
  digitalWrite(R_IN3, LOW); digitalWrite(R_IN4, LOW);
}

void applyPower(int in1, int in2, int channel, int pwm) {
  // Reversed software polarity to fix backwards movement
  if (pwm > 0) {
    digitalWrite(in1, LOW);  digitalWrite(in2, HIGH);
  } else if (pwm < 0) {
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  } else {
    digitalWrite(in1, LOW);  digitalWrite(in2, LOW);
  }
  
  int final_pwm = abs(pwm);
  if (final_pwm > 0 && final_pwm < PWM_MIN) final_pwm = PWM_MIN;
  if (final_pwm > PWM_MAX) final_pwm = PWM_MAX;
  
  ledcWrite(channel, final_pwm);
}

void setTargetVelocities(float v, float omega) {
  target_speed_L = v - omega * (WHEEL_BASE / 2.0f);
  target_speed_R = v + omega * (WHEEL_BASE / 2.0f);
}

// ═══════════════════════════════════════════════════════════════════════
//  SERIAL PARSING & COMMAND TIMEOUT
// ═══════════════════════════════════════════════════════════════════════
uint32_t last_cmd_ms = 0;
const uint32_t CMD_TIMEOUT_MS = 500;   
String serial_buf = "";

void parseSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      if (serial_buf.length() >= 3 && serial_buf[0] == 'C' && serial_buf[1] == ':') {
        int comma = serial_buf.indexOf(',', 2);
        if (comma > 2) {
          float v     = serial_buf.substring(2, comma).toFloat();
          float omega = serial_buf.substring(comma + 1).toFloat();
          setTargetVelocities(v, omega);
          last_cmd_ms = millis();
          digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
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
//  SETUP
// ═══════════════════════════════════════════════════════════════════════
void setup() {
  pinMode(STATUS_LED, OUTPUT); digitalWrite(STATUS_LED, LOW);
  motorInit();

  pinMode(ENC_L_A, INPUT); 
  pinMode(ENC_L_B, INPUT);
  
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrLeft,  RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrRight, RISING);

  Serial.begin(115200);

  for (int i = 0; i < 3; i++) {
    digitalWrite(STATUS_LED, HIGH); delay(150);
    digitalWrite(STATUS_LED, LOW);  delay(150);
  }
  Serial.println("SMALLBOT PID READY");
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

  if ((millis() - last_cmd_ms) > CMD_TIMEOUT_MS) {
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

  if (abs(target_speed_L) < 0.001) {
    applyPower(L_IN1, L_IN2, LEDC_CH_L, 0);
    error_sum_L = 0; 
  } else {
    float error_L = target_speed_L - current_speed_L;
    
    // Adaptively increase integral term faster if not moving
    float dynamic_Ki = (abs(current_speed_L) < 0.01) ? 300.0f : 100.0f;
    error_sum_L += error_L * dynamic_Ki * dt;
    error_sum_L = constrain(error_sum_L, -200.0f, 200.0f);
    
    // Start from 140 PWM and add PID
    float stiction_L = (target_speed_L > 0) ? 140.0f : -140.0f;
    float pwm_L = stiction_L + (150.0f * error_L) + error_sum_L;
    
    applyPower(L_IN1, L_IN2, LEDC_CH_L, (int)pwm_L);
  }

  if (abs(target_speed_R) < 0.001) {
    applyPower(R_IN3, R_IN4, LEDC_CH_R, 0);
    error_sum_R = 0; 
  } else {
    float error_R = target_speed_R - current_speed_R;
    
    // Adaptively increase integral term faster if not moving
    float dynamic_Ki = (abs(current_speed_R) < 0.01) ? 300.0f : 100.0f;
    error_sum_R += error_R * dynamic_Ki * dt;
    error_sum_R = constrain(error_sum_R, -200.0f, 200.0f);
    
    // Start from 140 PWM and add PID
    float stiction_R = (target_speed_R > 0) ? 140.0f : -140.0f;
    float pwm_R = stiction_R + (150.0f * error_R) + error_sum_R;
    
    applyPower(R_IN3, R_IN4, LEDC_CH_R, (int)pwm_R);
  }

  Serial.print("E:");
  Serial.print(ticks_L);
  Serial.print(',');
  Serial.println(ticks_R);
}
