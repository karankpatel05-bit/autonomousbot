/*
 * ╔══════════════════════════════════════════════════════════════════════╗
 * ║  SMALLBOT — ESP32 Firmware  (SMART PID - FIXED INTERRUPTS)           ║
 * ╠══════════════════════════════════════════════════════════════════════╣
 * ║  Reverted to RISING edge interrupts to fix stuck encoder readings.   ║
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
//  LEDC (hardware PWM) - v2.x Format
// ═══════════════════════════════════════════════════════════════════════
#define LEDC_FREQ       20000   
#define LEDC_RES        8       
#define LEDC_CH_L       0       
#define LEDC_CH_R       1       

// ═══════════════════════════════════════════════════════════════════════
//  EXACT HARDWARE CONSTANTS (MEASURED VIA RISING EDGE)
// ═══════════════════════════════════════════════════════════════════════
const float TPR_L       = 349.0f;  // Your exact measured RISING ticks
const float TPR_R       = 362.0f;  // Your exact measured RISING ticks
const float WHEEL_DIA   = 0.043f;  
const float WHEEL_BASE  = 0.140f;  

const float WHEEL_CIRC  = PI * WHEEL_DIA; 

const int   PWM_MIN     = 55;      
const int   PWM_MAX     = 255;     

// ═══════════════════════════════════════════════════════════════════════
//  ENCODER STATE
// ═══════════════════════════════════════════════════════════════════════
volatile int32_t enc_left  = 0;
volatile int32_t enc_right = 0;

// Reverted direction checks back to standard logic matching RISING edge
void IRAM_ATTR isrLeft()  { 
  if (digitalRead(ENC_L_B) == HIGH) enc_left++; else enc_left--; 
}
void IRAM_ATTR isrRight() { 
  if (digitalRead(ENC_R_B) == HIGH) enc_right--; else enc_right++; // Mirrored
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
  if (pwm > 0) {
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  } else if (pwm < 0) {
    digitalWrite(in1, LOW);  digitalWrite(in2, HIGH);
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

  // Pins 34 and 35 don't have software pullups, but setting them to INPUT explicitly
  pinMode(ENC_L_A, INPUT); 
  pinMode(ENC_L_B, INPUT);
  
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);
  
  // CHANGED back to RISING edge (this is what worked perfectly for you)
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrLeft,  RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrRight, RISING);

  Serial.begin(115200);

  for (int i = 0; i < 3; i++) {
    digitalWrite(STATUS_LED, HIGH); delay(150);
    digitalWrite(STATUS_LED, LOW);  delay(150);
  }
  Serial.println("SMALLBOT PID READY (FIXED INTERRUPTS)");
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
  float dt = (now - last_loop_ms) / 1000.0f; 

  if (dt >= (LOOP_PERIOD_MS / 1000.0f)) {
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
      error_sum_L += error_L * dt;
      float pwm_L = (target_speed_L * 400.0f) + (Kp * error_L) + (Ki * error_sum_L);
      applyPower(L_IN1, L_IN2, LEDC_CH_L, (int)pwm_L);
    }

    if (abs(target_speed_R) < 0.001) {
      applyPower(R_IN3, R_IN4, LEDC_CH_R, 0);
      error_sum_R = 0; 
    } else {
      float error_R = target_speed_R - current_speed_R;
      error_sum_R += error_R * dt;
      float pwm_R = (target_speed_R * 400.0f) + (Kp * error_R) + (Ki * error_sum_R);
      applyPower(R_IN3, R_IN4, LEDC_CH_R, (int)pwm_R);
    }

    Serial.print("E:");
    Serial.print(ticks_L);
    Serial.print(',');
    Serial.println(ticks_R);

    last_loop_ms = now;
  }
}
