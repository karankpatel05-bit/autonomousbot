/*
 * ╔══════════════════════════════════════════════════════════════════════╗
 * ║  SMALLBOT — Arduino Uno Motor Base Firmware (PID Velocity Control)  ║
 * ╠══════════════════════════════════════════════════════════════════════╣
 * ║  Motor Driver : PWM / DIR / EL  (Enable-Brake style)               ║
 * ║  Encoders     : Single-channel (A-pin only); direction from DIR pin ║
 * ║  Serial       : 115200 baud, USB to Raspberry Pi                   ║
 * ║  PWM ceiling  : 50  (hardware maximum — never exceeded)            ║
 * ╚══════════════════════════════════════════════════════════════════════╝
 *
 * SERIAL PROTOCOL (matches ESP32 bridge node):
 *   RPI → Arduino  :  "C:<v>,<omega>\n"
 *                       v     = linear  velocity m/s  (float)
 *                       omega = angular velocity rad/s (float)
 *   RPI → Arduino  :  "R\n"   — reset encoder counts + PID integrators
 *
 *   Arduino → RPI  :  "E,<leftTicks>,<rightTicks>\n"
 *                       Cumulative encoder ticks, sent every 100 ms
 *
 * ENCODER DIRECTION SENSING:
 *   Unlike the ESP32 which used dual-channel quadrature (A+B pins),
 *   this Arduino uses single-channel encoders (A-pin only).
 *   Direction is inferred from the motor DIR output pin at ISR time:
 *     Left  motor: DIR LOW  → forward → enc_left++
 *     Right motor: DIR HIGH → forward → enc_right++
 */

// ==========================================
// PIN DEFINITIONS
// ==========================================

// Encoder interrupt pins (MUST be 2 and 3 on Arduino Uno)
const int LEFT_ENCODER  = 2;   // A-pin of left  encoder → INT0
const int RIGHT_ENCODER = 3;   // A-pin of right encoder → INT1

// Left motor
const int LEFT_PWM  = 5;   // VR  — speed (analogWrite)
const int LEFT_DIR  = 4;   // Z/F — direction (HIGH = backward)
const int LEFT_EL   = 8;   // EL  — enable/brake (HIGH = run, LOW = brake)

// Right motor
const int RIGHT_PWM = 6;   // VR  — speed (analogWrite)
const int RIGHT_DIR = 12;  // Z/F — direction (HIGH = forward for right motor)
const int RIGHT_EL  = 9;   // EL  — enable/brake

// ==========================================
// HARDWARE CONSTANTS
// ==========================================

const float TPR_L      = 349.0f;   // ticks / revolution — LEFT  wheel (measured)
const float TPR_R      = 362.0f;   // ticks / revolution — RIGHT wheel (measured)
const float WHEEL_DIA  = 0.150f;   // wheel diameter  (m)
const float WHEEL_BASE = 0.400f;   // wheel-centre to wheel-centre (m)
const float WHEEL_CIRC = PI * WHEEL_DIA;  // circumference (m) ≈ 0.13509 m

// PWM limits — HARD CAP: no command can ever exceed PWM_MAX = 50
const int PWM_MIN = 15;   // minimum PWM to overcome motor stiction (tune if needed)
const int PWM_MAX = 50;   // ABSOLUTE MAXIMUM — corresponds to '1' in test code

// ==========================================
// ENCODER STATE  (volatile — updated inside ISR)
// ==========================================

volatile long enc_left  = 0;   // cumulative ticks, left  wheel
volatile long enc_right = 0;   // cumulative ticks, right wheel

// ==========================================
// PID STATE
// ==========================================

// Target wheel speeds in m/s — set by parseSerial() from C:<v>,<omega>
float target_speed_L = 0.0f;
float target_speed_R = 0.0f;

// Integral accumulators
float error_sum_L = 0.0f;
float error_sum_R = 0.0f;

// Previous tick snapshots for speed estimation
long prev_ticks_L = 0;
long prev_ticks_R = 0;

// ==========================================
// TIMING
// ==========================================

const unsigned long LOOP_PERIOD_MS = 50;    // 20 Hz PID loop
const unsigned long CMD_TIMEOUT_MS = 500;   // stop if no command for 500 ms
const unsigned long PRINT_PERIOD_MS = 100;  // send encoder data every 100 ms

unsigned long last_loop_ms  = 0;
unsigned long last_print_ms = 0;
unsigned long last_cmd_ms   = 0;

// ==========================================
// SERIAL BUFFER
// ==========================================

String serial_buf = "";

// ==========================================
// SETUP
// ==========================================

void setup() {
  Serial.begin(115200);

  // Motor output pins
  pinMode(LEFT_PWM,  OUTPUT);
  pinMode(LEFT_DIR,  OUTPUT);
  pinMode(LEFT_EL,   OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_DIR, OUTPUT);
  pinMode(RIGHT_EL,  OUTPUT);

  // Encoder input pins
  pinMode(LEFT_ENCODER,  INPUT);
  pinMode(RIGHT_ENCODER, INPUT);

  // Hardware interrupts — RISING edge only (single-channel)
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER),  leftEncoderISR,  RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER), rightEncoderISR, RISING);

  stopMotors();

  Serial.println("SMALLBOT ARDUINO PID READY");
  Serial.print("PWM_MAX="); Serial.println(PWM_MAX);
  Serial.print("WHEEL_BASE="); Serial.println(WHEEL_BASE, 3);
}

// ==========================================
// MAIN LOOP
// ==========================================

void loop() {
  // 1. Parse incoming serial commands from Raspberry Pi
  parseSerial();

  // 2. Command timeout watchdog — stop if RPI goes silent
  if ((millis() - last_cmd_ms) > CMD_TIMEOUT_MS) {
    target_speed_L = 0.0f;
    target_speed_R = 0.0f;
  }

  // 3. PID control loop @ 20 Hz
  unsigned long now = millis();
  if ((now - last_loop_ms) >= LOOP_PERIOD_MS) {
    float dt = (now - last_loop_ms) / 1000.0f;
    last_loop_ms = now;
    runPID(dt);
  }

  // 4. Send encoder telemetry @ 10 Hz
  if ((millis() - last_print_ms) >= PRINT_PERIOD_MS) {
    noInterrupts();
    long l = enc_left;
    long r = enc_right;
    interrupts();

    Serial.print("E,");
    Serial.print(l);
    Serial.print(",");
    Serial.println(r);

    last_print_ms = millis();
  }
}

// ==========================================
// ENCODER INTERRUPT SERVICE ROUTINES
// ==========================================
// Single-channel encoder: direction inferred from the motor DIR pin.
// This matches the wiring of the motor driver:
//   Left  motor:  DIR LOW  = forward → enc_left++
//   Right motor:  DIR HIGH = forward → enc_right++  (right motor is inverted)

volatile unsigned long last_left_isr = 0;
volatile unsigned long last_right_isr = 0;

void leftEncoderISR() {
  unsigned long now = micros();
  // 500us debounce to filter out high-frequency electrical noise
  if (now - last_left_isr > 500) {
    if (digitalRead(LEFT_DIR) == LOW) {
      enc_left++;
    } else {
      enc_left--;
    }
    last_left_isr = now;
  }
}

void rightEncoderISR() {
  unsigned long now = micros();
  // 500us debounce to filter out high-frequency electrical noise
  if (now - last_right_isr > 500) {
    if (digitalRead(RIGHT_DIR) == HIGH) {
      enc_right++;
    } else {
      enc_right--;
    }
    last_right_isr = now;
  }
}

// ==========================================
// SERIAL PARSER
// ==========================================

void parseSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      serial_buf.trim();

      // "C:<v>,<omega>" — velocity command
      if (serial_buf.length() >= 3 && serial_buf[0] == 'C' && serial_buf[1] == ':') {
        int comma = serial_buf.indexOf(',', 2);
        if (comma > 2) {
          float v     = serial_buf.substring(2, comma).toFloat();
          float omega = serial_buf.substring(comma + 1).toFloat();
          // Differential drive: compute individual wheel target speeds
          target_speed_L = v - omega * (WHEEL_BASE / 2.0f);
          target_speed_R = v + omega * (WHEEL_BASE / 2.0f);
          last_cmd_ms = millis();
        }
      }
      // "R" — reset encoders and PID state
      else if (serial_buf.length() > 0 && serial_buf[0] == 'R') {
        noInterrupts();
        enc_left  = 0;
        enc_right = 0;
        interrupts();
        prev_ticks_L = 0;
        prev_ticks_R = 0;
        error_sum_L  = 0.0f;
        error_sum_R  = 0.0f;
        target_speed_L = 0.0f;
        target_speed_R = 0.0f;
        Serial.println(">>> ENCODERS & PID RESET <<<");
      }

      serial_buf = "";
    } else if (c != '\r') {
      serial_buf += c;
      if (serial_buf.length() > 32) serial_buf = "";  // overflow guard
    }
  }
}

// ==========================================
// PID VELOCITY CONTROLLER
// ==========================================
/*
 * PI controller (no D term needed for low-speed motor control).
 *
 * For each wheel:
 *   1. Estimate current speed from encoder delta ticks over dt.
 *   2. Compute error = target_speed - current_speed.
 *   3. Use adaptive Ki: ramp up faster when the wheel is stalled.
 *   4. Add stiction feedforward so the motor starts moving immediately.
 *   5. Clamp output to [PWM_MIN, PWM_MAX] with correct sign.
 *
 * All PWM values are capped at PWM_MAX (50) — the hardware maximum.
 */

void runPID(float dt) {
  // Snapshot encoder counts
  noInterrupts();
  long ticks_L = enc_left;
  long ticks_R = enc_right;
  interrupts();

  long delta_L = ticks_L - prev_ticks_L;
  long delta_R = ticks_R - prev_ticks_R;
  prev_ticks_L = ticks_L;
  prev_ticks_R = ticks_R;

  // Estimated wheel speeds (m/s)
  float current_speed_L = (delta_L / TPR_L) * WHEEL_CIRC / dt;
  float current_speed_R = (delta_R / TPR_R) * WHEEL_CIRC / dt;

  // ── Left wheel ─────────────────────────────────────────────────────────
  if (abs(target_speed_L) < 0.001f) {
    applyPower(LEFT_DIR, LEFT_EL, LEFT_PWM, 0);
    error_sum_L = 0.0f;
  } else {
    float error_L = target_speed_L - current_speed_L;

    // Adaptive integral: ramp up faster when stalled (wheel not turning yet)
    float dynamic_Ki_L = (abs(current_speed_L) < 0.01f) ? 60.0f : 20.0f;
    error_sum_L += error_L * dynamic_Ki_L * dt;
    error_sum_L  = constrain(error_sum_L, -40.0f, 40.0f);

    // Stiction feedforward: start near PWM_MIN so the motor always moves
    float stiction_L = (target_speed_L > 0) ? (float)PWM_MIN : -(float)PWM_MIN;
    // Proportional + integral around the stiction base
    float pwm_L = stiction_L + (30.0f * error_L) + error_sum_L;

    applyPower(LEFT_DIR, LEFT_EL, LEFT_PWM, (int)pwm_L);
  }

  // ── Right wheel ────────────────────────────────────────────────────────
  if (abs(target_speed_R) < 0.001f) {
    applyPower(RIGHT_DIR, RIGHT_EL, RIGHT_PWM, 0);
    error_sum_R = 0.0f;
  } else {
    float error_R = target_speed_R - current_speed_R;

    float dynamic_Ki_R = (abs(current_speed_R) < 0.01f) ? 60.0f : 20.0f;
    error_sum_R += error_R * dynamic_Ki_R * dt;
    error_sum_R  = constrain(error_sum_R, -40.0f, 40.0f);

    float stiction_R = (target_speed_R > 0) ? (float)PWM_MIN : -(float)PWM_MIN;
    float pwm_R = stiction_R + (30.0f * error_R) + error_sum_R;

    applyPower(RIGHT_DIR, RIGHT_EL, RIGHT_PWM, (int)pwm_R);
  }
}

// ==========================================
// MOTOR DRIVER INTERFACE
// ==========================================
/*
 * applyPower() — set motor direction and PWM, respecting the PIN polarity:
 *   Left  motor:  forward = DIR LOW,  backward = DIR HIGH
 *   Right motor:  forward = DIR HIGH, backward = DIR LOW  (inverted)
 *
 * The dirPin, elPin, pwmPin arguments make this function usable for both
 * left and right motors without code duplication.
 *
 * PWM is HARD-CLAMPED to [-PWM_MAX, PWM_MAX].
 * Values between 0 and PWM_MIN are bumped up to PWM_MIN so the motor
 * does not stall in the dead zone.
 */

void applyPower(int dirPin, int elPin, int pwmPin, int pwm) {
  // Hard-clamp to maximum allowed
  pwm = constrain(pwm, -PWM_MAX, PWM_MAX);

  if (pwm == 0) {
    analogWrite(pwmPin, 0);
    digitalWrite(elPin, LOW);   // brake
    return;
  }

  // Dead-band: if requested but below stiction, bump to minimum
  int absPWM = abs(pwm);
  if (absPWM < PWM_MIN) absPWM = PWM_MIN;

  // Set direction — note the per-motor polarity is handled by the caller
  // providing the correct dirPin. The ISR direction inference must match.
  if (pwmPin == LEFT_PWM) {
    // Left motor: LOW = forward, HIGH = backward
    digitalWrite(dirPin, (pwm > 0) ? LOW : HIGH);
  } else {
    // Right motor: HIGH = forward, LOW = backward (inverted wiring)
    digitalWrite(dirPin, (pwm > 0) ? HIGH : LOW);
  }

  digitalWrite(elPin, HIGH);    // enable motor
  analogWrite(pwmPin, absPWM);  // apply speed — always <= PWM_MAX
}

// ==========================================
// STOP ALL MOTORS
// ==========================================

void stopMotors() {
  analogWrite(LEFT_PWM,  0);
  analogWrite(RIGHT_PWM, 0);
  digitalWrite(LEFT_EL,  LOW);
  digitalWrite(RIGHT_EL, LOW);
}
