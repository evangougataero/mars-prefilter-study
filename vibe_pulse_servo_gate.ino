/*
  File: vibe_pulse_servo_gate.ino

  Purpose
  -------
  Drive a vibration motor in repeated ON/OFF pulses while moving a servo “gate”
  to a brief CLOSED position right after each vibration pulse ends, then reopening
  shortly before the next vibration pulse begins.

  Typical use
  ----------
  Particle/dust feeder or hopper system:
    - Motor ON: agitate / dispense
    - Motor OFF: stop agitation
    - Servo CLOSE: briefly pinch/close to reduce dribble/backflow
    - Servo OPEN: reopen before the next agitation pulse

  Behavior per cycle
  ------------------
  1) Motor ON  for ON_MS
  2) Motor OFF
  3) Servo -> CLOSED immediately
  4) Wait (OFF_MS - REOPEN_LEAD_MS)
  5) Servo -> OPEN
  6) Wait REOPEN_LEAD_MS
  Repeat for PULSE_CYCLES cycles, then stop forever.

  Notes
  -----
  - On Arduino Uno, the Servo library uses Timer1, which can affect PWM on pins 9/10.
    BUZZER_PIN=11 avoids that conflict.
*/

#include <Servo.h>

// ================== USER SETTINGS ==================

// Pins
const int SERVO_PIN  = 6;    // Servo signal pin
const int MOTOR_PIN  = 11;   // PWM-capable pin for vibration motor driver (was BUZZER_PIN)

// Servo positions (degrees)
const int SERVO_OPEN_DEG  = 123;  // steady/open position
const int SERVO_CLOSE_DEG = 130;  // brief closed position

// Vibration motor pulse behavior
const uint8_t VIBE_PWM = 255;           // motor PWM when ON (0–255)
const unsigned long ON_MS  = 500UL;     // ON duration per cycle
const unsigned long OFF_MS = 1500UL;    // OFF duration per cycle
const int PULSE_CYCLES = 1100;          // number of ON/OFF cycles

// Reopen lead time: open this long before the next ON pulse starts
const unsigned long REOPEN_LEAD_MS = 250UL;

// Optional startup delay (lets you get ready / start logging)
const unsigned long STARTUP_DELAY_MS = 11000UL;

// ================== GLOBALS ==================
Servo gateServo;

// ================== HELPERS ==================
static void motor_set_pwm(uint8_t pwm) {
  analogWrite(MOTOR_PIN, pwm);
}

static void gate_open() {
  gateServo.write(SERVO_OPEN_DEG);
}

static void gate_close() {
  gateServo.write(SERVO_CLOSE_DEG);
}

// ================== SETUP ==================
void setup() {
  Serial.begin(9600);

  gateServo.attach(SERVO_PIN);
  gate_open();

  pinMode(MOTOR_PIN, OUTPUT);
  motor_set_pwm(0);

  delay(STARTUP_DELAY_MS);
  Serial.println(F("Starting vibration pulses with servo gating..."));
}

// ================== LOOP ==================
void loop() {
  // Guard: ensure timing logic never underflows
  const unsigned long quiet_before_reopen =
      (OFF_MS > REOPEN_LEAD_MS) ? (OFF_MS - REOPEN_LEAD_MS) : 0UL;
  const unsigned long quiet_after_reopen =
      (OFF_MS > REOPEN_LEAD_MS) ? REOPEN_LEAD_MS : OFF_MS;

  for (int i = 0; i < PULSE_CYCLES; i++) {
    Serial.print(F("Cycle "));
    Serial.print(i + 1);
    Serial.print(F(" of "));
    Serial.println(PULSE_CYCLES);

    // 1) Motor ON
    motor_set_pwm(VIBE_PWM);
    delay(ON_MS);

    // 2) Motor OFF
    motor_set_pwm(0);

    // 3) Close gate immediately when vibration ends
    gate_close();

    // 4) Stay closed until shortly before next ON
    delay(quiet_before_reopen);

    // 5) Reopen just before next ON
    gate_open();

    // 6) Wait remaining quiet time
    delay(quiet_after_reopen);
  }

  // Done: motor off, gate open (or change to close if you prefer)
  motor_set_pwm(0);
  gate_open();
  Serial.println(F("Done with vibration pulses. Halting."));

  while (true) { /* stop here */ }
}
