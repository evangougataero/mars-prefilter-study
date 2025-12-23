/*
  File: sdp8xx_mux_dp_logger_velocity.ino

  Purpose
  -------
  Log wind-tunnel differential pressure and derived velocity using multiple Sensirion
  SDP8xx differential pressure sensors.

  Output format (CSV)
  -------------------
  Prints exactly four comma-separated fields (no units) once per SAMPLE_MS window:

      t,dp1,dp2,vel

  where:
    - t   = elapsed seconds since experiment start
    - dp1 = averaged ΔP from "other" sensor (DP_OTHER_CH)   [Pa]
    - dp2 = averaged ΔP from the remaining sensor           [Pa]
    - vel = velocity derived ONLY from the FACE sensor      [m/s]
            FACE sensor should measure static-static ΔP across a contraction section.

  How it works
  ------------
  1) CALIBRATION:
     - Collect CAL_WINDOWS windows of samples with fan off (and valve closed).
     - Compute per-sensor offset (truncated to 0.01) and store as calOffset[].
  2) RUN:
     - Turn fan on, optionally open valve.
     - Every SAMPLE_MS window, average each sensor, subtract offsets, print CSV line.
     - Derive velocity from FACE ΔP using Bernoulli across an area change:
         Δp = 0.5 * ρ * v_small^2 * [1 - (Ds/Dl)^4]
       then optionally convert to large-duct velocity via continuity:
         v_large = v_small * (Ds/Dl)^2

  Notes
  -----
  - All SDP sensors are started in continuous averaging mode.
  - FACE_CH and DP_OTHER_CH are selected by multiplexer channel number (0–7).
  - If time series is later post-processed, keep this output strictly numeric CSV.
*/

#include <Wire.h>
#include <math.h>  // fabsf, sqrtf, floorf, ceilf, isnan

// ================== USER CONFIG ==================
#define MUX_ADDR     0x70
#define I2C_HZ_FAST  400000UL

// Physical wiring: SENSOR_CHANNELS[i] = TCA9548A channel for sensor i
const uint8_t SENSOR_CHANNELS[] = {7, 2, 0};  // CH7, CH2, CH0
const char*   SENSOR_LABELS[]   = {"Diff P sensor #1", "Diff P sensor #2", "Diff P sensor #3"};
const size_t  NUM_SENSORS = sizeof(SENSOR_CHANNELS) / sizeof(SENSOR_CHANNELS[0]);

// --- Assign roles by MUX channel number ---
// FACE_CH must be Δp across contraction (static-static), used to compute velocity
const uint8_t FACE_CH     = 2;  // set to 7/2/0
const uint8_t DP_OTHER_CH = 0;  // set to 7/2/0 (dp1). dp2 will be the remaining sensor.

// SDP8xx I2C address candidates (commonly 0x25, 0x26)
static const uint8_t SDP_ADDR_CANDIDATES[] = {0x25, 0x26};

// Fan control
const int fanPin = 3;
const unsigned long FAN_SPINUP_MS   = 5000UL;
const uint8_t       FAN_DUTY_NORMAL = 255;

// Optional pinch valve
#define ENABLE_PINCH_VALVE 1
const int  valvePin = 5;
const bool VALVE_ACTIVE_HIGH = true;

// Experiment duration
const unsigned long EXPERIMENT_SECONDS = 2200UL;

// Sampling window length
const unsigned long SAMPLE_MS = 2000UL;

// -------- Geometry & fluid constants --------
// Probe #1: small downstream duct (after contraction)
// Probe #2: large upstream duct (before contraction)
const float D_PROBE1_SMALL_IN = 1.142f;  // inches
const float D_PROBE2_LARGE_IN = 3.000f;  // inches
const float AIR_RHO           = 1.20f;   // kg/m^3

// Precompute Bernoulli coefficient for static-static across an area change:
// Δp = 0.5 * ρ * v_small^2 * [1 - (Ds/Dl)^4]
const float R_ds_dl      = D_PROBE1_SMALL_IN / D_PROBE2_LARGE_IN; // Ds/Dl
const float R2           = R_ds_dl * R_ds_dl;                     // (Ds/Dl)^2
const float R4           = R2 * R2;                               // (Ds/Dl)^4
const float ONE_MINUS_R4 = (1.0f - R4);

// Select which velocity you output (compile-time)
enum VelocityAt { VELOCITY_AT_PROBE1_SMALL, VELOCITY_AT_PROBE2_LARGE };
const VelocityAt VEL_AT = VELOCITY_AT_PROBE2_LARGE;

// ================== STATE ==================
enum RunState { CALIBRATING, RUNNING, DONE };
RunState state = CALIBRATING;

unsigned long windowStartMs = 0;
unsigned long runStartMs    = 0;

uint8_t sensorAddr[NUM_SENSORS] = {0};

// Accumulators per SAMPLE_MS window
float    sumPa[NUM_SENSORS]     = {0};
uint32_t countRead[NUM_SENSORS] = {0};

// Calibration
static const uint8_t CAL_WINDOWS = 3;
uint8_t  calWinIdx = 0;
float    calWinAvg[NUM_SENSORS][CAL_WINDOWS];
float    calOffset[NUM_SENSORS];

// Role indices resolved from channel choices
size_t FACE_IDX      = 0;  // Δp across contraction -> velocity source
size_t DP_OTHER_IDX  = 1;  // dp1
size_t DP_SECOND_IDX = 2;  // dp2

// ================== SDP CRC ==================
static bool sdp_crc_ok(uint8_t msb, uint8_t lsb, uint8_t crc) {
  uint8_t data[2] = {msb, lsb};
  uint8_t c = 0xFF;
  for (int i = 0; i < 2; i++) {
    c ^= data[i];
    for (int b = 0; b < 8; b++) {
      c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x31) : (uint8_t)(c << 1);
    }
  }
  return (c == crc);
}

// ================== MUX HELPERS ==================
static bool mux_select_channel(uint8_t ch) {
  if (ch > 7) return false;
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << ch);
  bool ok = (Wire.endTransmission() == 0);
  delayMicroseconds(500);
  return ok;
}

static bool mux_disable_all() {
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(0x00);
  return (Wire.endTransmission() == 0);
}

// ================== SDP HELPERS ==================
static bool sdp_start_continuous_dp_averaging(uint8_t addr) {
  // Command 0x3615: start continuous differential pressure with averaging
  Wire.beginTransmission(addr);
  Wire.write((uint8_t)0x36);
  Wire.write((uint8_t)0x15);
  return (Wire.endTransmission() == 0);
}

static bool sdp_read_dp_pa(uint8_t addr, float &dpPa) {
  // Reads 2-byte dp + CRC (3 bytes total)
  uint8_t n = Wire.requestFrom(addr, (uint8_t)3);
  if (n != 3) return false;

  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();
  uint8_t crc = Wire.read();
  if (!sdp_crc_ok(msb, lsb, crc)) return false;

  int16_t raw = (int16_t)((msb << 8) | lsb);
  dpPa = raw / 60.0f;  // SDP810-500Pa scaling
  return true;
}

// ================== UTIL ==================
static int index_for_channel(uint8_t ch) {
  for (size_t i = 0; i < NUM_SENSORS; i++) {
    if (SENSOR_CHANNELS[i] == ch) return (int)i;
  }
  return -1;
}

static void resolve_role_indices() {
  // FACE
  int face_i = index_for_channel(FACE_CH);
  FACE_IDX = (face_i >= 0) ? (size_t)face_i : 0;

  // remaining indices (not FACE)
  size_t remaining[NUM_SENSORS];
  size_t rcount = 0;
  for (size_t i = 0; i < NUM_SENSORS; i++) {
    if (i != FACE_IDX) remaining[rcount++] = i;
  }

  // dp1 preference
  int pref_i = index_for_channel(DP_OTHER_CH);
  if (pref_i >= 0 && (size_t)pref_i != FACE_IDX) {
    DP_OTHER_IDX = (size_t)pref_i;
  } else {
    DP_OTHER_IDX = (rcount > 0) ? remaining[0] : FACE_IDX;
  }

  // dp2 = the other remaining
  if (rcount > 1) {
    DP_SECOND_IDX = (remaining[0] == DP_OTHER_IDX) ? remaining[1] : remaining[0];
  } else {
    DP_SECOND_IDX = DP_OTHER_IDX; // fallback (will duplicate)
  }
}

static void reset_window_accumulators() {
  for (size_t i = 0; i < NUM_SENSORS; i++) {
    sumPa[i] = 0.0f;
    countRead[i] = 0;
  }
}

static float trunc_hundredths(float x) {
  return (x >= 0.0f) ? floorf(x * 100.0f) / 100.0f : ceilf(x * 100.0f) / 100.0f;
}

// ---- Velocity helpers ----
static float v_small_from_dp_staticstatic(float dpPa_signed) {
  if (ONE_MINUS_R4 <= 0.0f) return NAN;
  float dp = fabsf(dpPa_signed);
  return sqrtf((2.0f * dp) / (AIR_RHO * ONE_MINUS_R4)); // m/s (small duct)
}

static float v_large_from_v_small(float v_small) {
  if (isnan(v_small)) return NAN;
  return v_small * R2; // v_large = v_small * (Ds/Dl)^2
}

static uint8_t detect_sdp_addr_on_active_bus() {
  for (uint8_t a : SDP_ADDR_CANDIDATES) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) return a;
  }
  return 0;
}

#if ENABLE_PINCH_VALVE
static void valve_set(bool open) {
  bool level = VALVE_ACTIVE_HIGH ? open : !open;
  digitalWrite(valvePin, level ? HIGH : LOW);
}
#endif

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  Wire.begin();
  Wire.setClock(I2C_HZ_FAST);
  delay(100);

#if ENABLE_PINCH_VALVE
  pinMode(valvePin, OUTPUT);
  valve_set(false); // closed during calibration
#endif

  pinMode(fanPin, OUTPUT);
  analogWrite(fanPin, 0);

  // Detect sensors and start continuous mode
  for (size_t i = 0; i < NUM_SENSORS; i++) {
    uint8_t ch = SENSOR_CHANNELS[i];
    if (!mux_select_channel(ch)) continue;

    uint8_t addr = detect_sdp_addr_on_active_bus();
    sensorAddr[i] = addr;

    if (addr != 0) {
      if (sdp_start_continuous_dp_averaging(addr)) {
        delay(10);
        float tmp;
        (void)sdp_read_dp_pa(addr, tmp); // flush first read
      }
    }
  }
  mux_disable_all();

  resolve_role_indices();

  // Human-readable role summary
  Serial.println(F("Role assignment:"));
  Serial.print(F("  FACE (velocity): ")); Serial.println(SENSOR_LABELS[FACE_IDX]);
  Serial.print(F("  dp1: "));             Serial.println(SENSOR_LABELS[DP_OTHER_IDX]);
  Serial.print(F("  dp2: "));             Serial.println(SENSOR_LABELS[DP_SECOND_IDX]);

  windowStartMs = millis();
  reset_window_accumulators();
}

// ================== LOOP ==================
void loop() {
  unsigned long now = millis();

  // Sample as fast as possible into the current window
  for (size_t i = 0; i < NUM_SENSORS; i++) {
    uint8_t ch   = SENSOR_CHANNELS[i];
    uint8_t addr = sensorAddr[i];
    if (addr == 0) continue;

    if (!mux_select_channel(ch)) continue;
    delayMicroseconds(500);

    float dpPa;
    if (sdp_read_dp_pa(addr, dpPa)) {
      if (state == RUNNING) dpPa -= calOffset[i];
      sumPa[i] += dpPa;
      countRead[i] += 1;
    }
    delayMicroseconds(150);
  }
  mux_disable_all();

  // End of window: compute averages, then either calibrate or print a CSV line
  if (now - windowStartMs >= SAMPLE_MS) {
    float winAvg[NUM_SENSORS];
    for (size_t i = 0; i < NUM_SENSORS; i++) {
      winAvg[i] = (countRead[i] > 0) ? (sumPa[i] / (float)countRead[i]) : NAN;
    }

    if (state == CALIBRATING) {
      // Store calibration window averages
      for (size_t i = 0; i < NUM_SENSORS; i++) {
        calWinAvg[i][calWinIdx] = isnan(winAvg[i]) ? 0.0f : winAvg[i];
      }

      calWinIdx++;

      if (calWinIdx >= CAL_WINDOWS) {
        // Compute offsets (truncated to hundredths)
        for (size_t i = 0; i < NUM_SENSORS; i++) {
          float m = 0.0f;
          for (uint8_t w = 0; w < CAL_WINDOWS; w++) m += calWinAvg[i][w];
          m /= (float)CAL_WINDOWS;
          calOffset[i] = trunc_hundredths(m);
        }

        // Human-readable calibration prints (not CSV)
        Serial.println(F("Calibration offsets:"));
        for (size_t i = 0; i < NUM_SENSORS; i++) {
          Serial.print(F("  "));
          Serial.print(SENSOR_LABELS[i]);
          Serial.print(F(": "));
          Serial.println(calOffset[i], 2);
        }

        // Spin up fan and start experiment
        analogWrite(fanPin, 255);
        delay(FAN_SPINUP_MS);
        analogWrite(fanPin, FAN_DUTY_NORMAL);

#if ENABLE_PINCH_VALVE
        valve_set(true);
#endif

        Serial.println(F("Experiment start"));
        Serial.println(F("t,dp1,dp2,vel")); // CSV header

        state = RUNNING;
        runStartMs = millis();
      }

    } else if (state == RUNNING) {
      float dp1     = winAvg[DP_OTHER_IDX];
      float dp2     = winAvg[DP_SECOND_IDX];
      float dp_face = winAvg[FACE_IDX];

      // Compute velocities from FACE Δp only
      float v_small = v_small_from_dp_staticstatic(dp_face);
      float v_large = v_large_from_v_small(v_small);
      float v_out   = (VEL_AT == VELOCITY_AT_PROBE1_SMALL) ? v_small : v_large;

      unsigned long elapsed_s = (millis() - runStartMs) / 1000UL;

      // Strict CSV line: t,dp1,dp2,vel
      Serial.print(elapsed_s);
      Serial.print(F(","));
      if (isnan(dp1)) Serial.print(F("nan")); else Serial.print(dp1, 1);
      Serial.print(F(","));
      if (isnan(dp2)) Serial.print(F("nan")); else Serial.print(dp2, 1);
      Serial.print(F(","));
      if (isnan(v_out)) Serial.print(F("nan")); else Serial.print(v_out, 3);
      Serial.println();

      if (elapsed_s >= EXPERIMENT_SECONDS) {
        analogWrite(fanPin, 0);
#if ENABLE_PINCH_VALVE
        valve_set(false);
#endif
        state = DONE;
      }
    }

    windowStartMs += SAMPLE_MS;
    reset_window_accumulators();
  }

  if (state == DONE) {
    while (true) delay(1000);
  }
}
