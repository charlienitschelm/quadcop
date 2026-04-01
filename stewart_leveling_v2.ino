// ═══════════════════════════════════════════════════════════════════════
// STEWART PLATFORM — ACTIVE LEVELING (v2)
// ═══════════════════════════════════════════════════════════════════════
// 
// APPROACH: Incremental closed-loop control.
//   Instead of computing an absolute target from IMU error (which requires
//   a perfect sensitivity constant), we INCREMENTALLY WALK the targets
//   toward level. Each loop, the target nudges a small amount proportional
//   to IMU error. The IMU stays in the loop — it keeps pushing until it
//   ACTUALLY reads level, regardless of geometry or sensitivity errors.
//
//   This fundamentally cannot overshoot because:
//   - Targets only move a few counts per loop (rate-limited by GAIN*dt)
//   - As the plate levels, error shrinks → nudge shrinks → natural damping
//   - When level, error=0 → nudge=0 → targets stop walking
//
// STARTUP SEQUENCE:
//   1. Stiction calibration (individual, sequential)
//   2. Go to mid stroke
//   3. Auto-detect IMU axis mapping (which actuator affects which IMU axis)
//   4. Lock IMU reference once stable
//   5. Active leveling loop
//
// ═══════════════════════════════════════════════════════════════════════

#include <Wire.h>
#include <MPU6050.h>

MPU6050 imu;

// ─── PIN ASSIGNMENTS ───────────────────────────────────────────────
const int DIR_1 = 2,  PWM_1 = 3,  POT_1 = A3;
const int DIR_2 = 4,  PWM_2 = 5,  POT_2 = A4;
const int DIR_3 = 6,  PWM_3 = 7,  POT_3 = A5;

// ─── STROKE CALIBRATION (measured) ─────────────────────────────────
const int RET_1 = 680, EXT_1 = 71, MID_1 = 375;
const int RET_2 = 673, EXT_2 = 68, MID_2 = 370;
const int RET_3 = 670, EXT_3 = 73, MID_3 = 371;

// ─── SAFE OPERATING RANGE ──────────────────────────────────────────
// Never command actuators outside these limits — 50+ count margin
const int SAFE_MIN = 130;
const int SAFE_MAX = 620;

// ─── TUNING ────────────────────────────────────────────────────────
// GAIN: how fast targets walk toward level (counts per accel-count per second)
// Higher = faster correction but more oscillation risk.
// Lower  = smoother but slower.
// 0.25 should correct 5 deg in ~2-3 seconds. 
// If too slow increase to 0.35. If oscillates reduce to 0.15.
float GAIN = 0.8;

// IMU low-pass filter — lower = smoother, less vibration noise
// 0.18 is a good balance between responsiveness and noise rejection.
const float ALPHA = 0.18;

// Hysteresis thresholds (in approximate degrees)
const float ACCEL_PER_DEG = 60.0;   // empirical for upside-down mounting
const float START_DEG     = 2.0;    // start correcting above this
const float STOP_DEG      = 1.0;    // stop correcting below this

// Actuator deadband — stop driving when within this many counts of target
const int ACT_DEADBAND = 8;

// ─── STICTION VALUES (measured at startup) ─────────────────────────
int STICTION_1 = 130;
int STICTION_2 = 105;
int STICTION_3 = 45;

// ─── DIRECTION MAPPING (auto-detected at startup) ──────────────────
// These map IMU error → actuator correction direction
// responseX[i] = how much IMU ax changes when actuator i extends 1 count
// responseY[i] = how much IMU ay changes when actuator i extends 1 count
float responseX[3] = {0, 0, 0};
float responseY[3] = {0, 0, 0};

// ─── STATE ─────────────────────────────────────────────────────────
float smoothAx, smoothAy;
float refAx, refAy;
float target1, target2, target3;  // float targets for smooth incrementing
bool  correcting      = false;
unsigned long prevTime = 0;
unsigned long lastPrint = 0;

// ═══════════════════════════════════════════════════════════════════════
// UTILITIES
// ═══════════════════════════════════════════════════════════════════════

void stopAll() {
  analogWrite(PWM_1, 0);
  analogWrite(PWM_2, 0);
  analogWrite(PWM_3, 0);
}

// Read stable IMU average over N samples
void readIMUAvg(float &outAx, float &outAy, int n = 40) {
  float sx = 0, sy = 0;
  for (int i = 0; i < n; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sx += ax;
    sy += ay;
    delay(8);
  }
  outAx = sx / n;
  outAy = sy / n;
}

// Update smoothed IMU with spike rejection
void updateIMU() {
  int16_t ax, ay, az, gx, gy, gz;
  imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  if (abs(ax - smoothAx) < 2500) {
    smoothAx = ALPHA * ax + (1.0f - ALPHA) * smoothAx;
  }
  if (abs(ay - smoothAy) < 2500) {
    smoothAy = ALPHA * ay + (1.0f - ALPHA) * smoothAy;
  }
}

// ═══════════════════════════════════════════════════════════════════════
// ACTUATOR DRIVE — proportional speed, always above stiction
// ═══════════════════════════════════════════════════════════════════════

void driveActuator(int dirPin, int pwmPin, int potPin,
                   int target, int stiction) {
  target = constrain(target, SAFE_MIN, SAFE_MAX);
  int pos = analogRead(potPin);
  if (pos < 50 || pos > 710) { analogWrite(pwmPin, 0); return; }

  int err = target - pos;
  int a   = abs(err);

  if (a <= ACT_DEADBAND) {
    analogWrite(pwmPin, 0);
    return;
  }

  // Proportional speed zones — always above stiction floor
  // More aggressive far away for faster tracking
  int spd;
  if      (a > 100) spd = stiction + 90;   // far — fast
  else if (a >  60) spd = stiction + 60;
  else if (a >  35) spd = stiction + 35;
  else if (a >  15) spd = stiction + 20;
  else              spd = stiction + 10;

  spd = constrain(spd, stiction + 5, 220);

  // HIGH = retract (count up), LOW = extend (count down)
  digitalWrite(dirPin, (err > 0) ? HIGH : LOW);
  analogWrite(pwmPin, spd);
}

// Blocking move — waits until all 3 reach targets
void moveToAndWait(int t1, int t2, int t3) {
  t1 = constrain(t1, SAFE_MIN, SAFE_MAX);
  t2 = constrain(t2, SAFE_MIN, SAFE_MAX);
  t3 = constrain(t3, SAFE_MIN, SAFE_MAX);

  unsigned long timeout = millis() + 10000;
  while (millis() < timeout) {
    int p1 = analogRead(POT_1), p2 = analogRead(POT_2), p3 = analogRead(POT_3);
    bool d1 = abs(t1 - p1) <= ACT_DEADBAND;
    bool d2 = abs(t2 - p2) <= ACT_DEADBAND;
    bool d3 = abs(t3 - p3) <= ACT_DEADBAND;
    if (d1 && d2 && d3) break;
    if (!d1) driveActuator(DIR_1, PWM_1, POT_1, t1, STICTION_1);
    else     analogWrite(PWM_1, 0);
    if (!d2) driveActuator(DIR_2, PWM_2, POT_2, t2, STICTION_2);
    else     analogWrite(PWM_2, 0);
    if (!d3) driveActuator(DIR_3, PWM_3, POT_3, t3, STICTION_3);
    else     analogWrite(PWM_3, 0);
    delay(8);
  }
  stopAll();
  delay(100);
}

// ═══════════════════════════════════════════════════════════════════════
// STICTION CALIBRATION — individual, sequential
// ═══════════════════════════════════════════════════════════════════════

int measureStiction(const char* name, int dirPin, int pwmPin, int potPin) {
  Serial.print("  "); Serial.print(name); Serial.print(": ");

  // Retract fully to known position
  digitalWrite(dirPin, HIGH);
  analogWrite(pwmPin, 220);
  delay(3500);
  analogWrite(pwmPin, 0);
  delay(500);

  // Ramp PWM from low until pot changes
  int baseline = analogRead(potPin);
  for (int pwm = 20; pwm <= 220; pwm += 3) {
    digitalWrite(dirPin, LOW);  // extend
    analogWrite(pwmPin, pwm);
    delay(180);
    int cur = analogRead(potPin);
    if (cur - baseline > 8) {
      analogWrite(pwmPin, 0);
      delay(200);
      Serial.print("stiction="); Serial.println(pwm);
      return pwm;
    }
    baseline = cur;
  }
  analogWrite(pwmPin, 0);
  Serial.println("not found — using 180");
  return 180;
}

// ═══════════════════════════════════════════════════════════════════════
// AUTO-DETECT DIRECTION MAPPING
// 
// We move each actuator by a known amount and measure how the IMU
// responds. This gives us the exact mapping between actuator movement
// and IMU axes — no guessing required.
// ═══════════════════════════════════════════════════════════════════════

void detectDirectionMapping() {
  Serial.println(">> Auto-detecting IMU axis mapping...");

  const int TEST_MOVE = 40;  // extend each actuator 40 counts from mid

  // Read baseline IMU at mid position
  float baseAx, baseAy;
  readIMUAvg(baseAx, baseAy, 60);
  Serial.print("  Baseline: ax="); Serial.print(baseAx, 0);
  Serial.print(" ay="); Serial.println(baseAy, 0);

  // --- Test actuator 1 ---
  int target1test = MID_1 - TEST_MOVE;  // extend (lower count)
  moveToAndWait(target1test, MID_2, MID_3);
  delay(300);
  float ax1, ay1;
  readIMUAvg(ax1, ay1, 60);
  responseX[0] = (ax1 - baseAx) / TEST_MOVE;
  responseY[0] = (ay1 - baseAy) / TEST_MOVE;
  Serial.print("  A1: dax="); Serial.print(ax1 - baseAx, 0);
  Serial.print("  day="); Serial.println(ay1 - baseAy, 0);
  moveToAndWait(MID_1, MID_2, MID_3);
  delay(300);

  // --- Test actuator 2 ---
  int target2test = MID_2 - TEST_MOVE;
  moveToAndWait(MID_1, target2test, MID_3);
  delay(300);
  float ax2, ay2;
  readIMUAvg(ax2, ay2, 60);
  responseX[1] = (ax2 - baseAx) / TEST_MOVE;
  responseY[1] = (ay2 - baseAy) / TEST_MOVE;
  Serial.print("  A2: dax="); Serial.print(ax2 - baseAx, 0);
  Serial.print("  day="); Serial.println(ay2 - baseAy, 0);
  moveToAndWait(MID_1, MID_2, MID_3);
  delay(300);

  // --- Test actuator 3 ---
  int target3test = MID_3 - TEST_MOVE;
  moveToAndWait(MID_1, MID_2, target3test);
  delay(300);
  float ax3, ay3;
  readIMUAvg(ax3, ay3, 60);
  responseX[2] = (ax3 - baseAx) / TEST_MOVE;
  responseY[2] = (ay3 - baseAy) / TEST_MOVE;
  Serial.print("  A3: dax="); Serial.print(ax3 - baseAx, 0);
  Serial.print("  day="); Serial.println(ay3 - baseAy, 0);
  moveToAndWait(MID_1, MID_2, MID_3);
  delay(300);

  // Verify we got meaningful responses
  float totalResponse = abs(responseX[0]) + abs(responseY[0]) +
                        abs(responseX[1]) + abs(responseY[1]) +
                        abs(responseX[2]) + abs(responseY[2]);

  if (totalResponse < 0.5) {
    Serial.println("!! WARNING: IMU barely responded to actuator movement.");
    Serial.println("   Check IMU mounting. Using fallback geometry.");
    // Fallback: assume standard 120-degree arrangement
    responseX[0] =  1.0; responseY[0] =  0.0;
    responseX[1] = -0.5; responseY[1] =  0.866;
    responseX[2] = -0.5; responseY[2] = -0.866;
  }

  Serial.println(">> Direction mapping complete:");
  Serial.print("  A1: rx="); Serial.print(responseX[0], 3);
  Serial.print(" ry="); Serial.println(responseY[0], 3);
  Serial.print("  A2: rx="); Serial.print(responseX[1], 3);
  Serial.print(" ry="); Serial.println(responseY[1], 3);
  Serial.print("  A3: rx="); Serial.print(responseX[2], 3);
  Serial.print(" ry="); Serial.println(responseY[2], 3);
}

// ═══════════════════════════════════════════════════════════════════════
// IMU STABLE LOCK — wait for variance to drop before locking reference
// ═══════════════════════════════════════════════════════════════════════

void lockIMUReference() {
  Serial.println(">> Hold plate LEVEL and STILL — locking reference...");
  const int   W       = 40;
  const float MAX_STD = 70.0;
  float bX[W], bY[W];
  int   idx  = 0;
  bool  full = false;
  unsigned long timeout = millis() + 20000;
  unsigned long lp = 0;

  while (millis() < timeout) {
    int16_t ax, ay, az, gx, gy, gz;
    imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    bX[idx] = ax;
    bY[idx] = ay;
    idx = (idx + 1) % W;
    if (idx == 0) full = true;

    if (full) {
      float mX = 0, mY = 0;
      for (int i = 0; i < W; i++) { mX += bX[i]; mY += bY[i]; }
      mX /= W; mY /= W;
      float vX = 0, vY = 0;
      for (int i = 0; i < W; i++) {
        vX += (bX[i] - mX) * (bX[i] - mX);
        vY += (bY[i] - mY) * (bY[i] - mY);
      }
      float sX = sqrt(vX / W), sY = sqrt(vY / W);

      if (millis() - lp >= 600) {
        lp = millis();
        Serial.print("  std="); Serial.print(sX, 0);
        Serial.print("/"); Serial.print(sY, 0);
        Serial.print("  ax="); Serial.print(mX, 0);
        Serial.print("  ay="); Serial.println(mY, 0);
      }

      if (sX < MAX_STD && sY < MAX_STD) {
        refAx    = mX;
        refAy    = mY;
        smoothAx = refAx;
        smoothAy = refAy;
        Serial.print(">> REFERENCE LOCKED  ax="); Serial.print(refAx, 0);
        Serial.print("  ay="); Serial.println(refAy, 0);
        return;
      }
    }
    delay(15);
  }
  // Timeout fallback
  float mX = 0, mY = 0;
  for (int i = 0; i < W; i++) { mX += bX[i]; mY += bY[i]; }
  refAx = mX / W; refAy = mY / W;
  smoothAx = refAx; smoothAy = refAy;
  Serial.println(">> Timeout — using best estimate");
}

// ═══════════════════════════════════════════════════════════════════════
// COMPUTE CORRECTION — how much to nudge each actuator
//
// Uses the measured response matrix to project IMU error onto each
// actuator. The dot product tells us how much THIS actuator contributed
// to THIS error — so we know how much to move it to fix the error.
// ═══════════════════════════════════════════════════════════════════════

void computeNudge(float errX, float errY, float dt,
                  float &nudge1, float &nudge2, float &nudge3) {
  // Project error onto each actuator's response direction
  // Dot product: how aligned is this actuator with the error?
  float proj1 = responseX[0] * errX + responseY[0] * errY;
  float proj2 = responseX[1] * errX + responseY[1] * errY;
  float proj3 = responseX[2] * errX + responseY[2] * errY;

  // Normalize by magnitude of response vector so all actuators
  // contribute equally regardless of their individual sensitivity
  float mag1 = responseX[0] * responseX[0] + responseY[0] * responseY[0];
  float mag2 = responseX[1] * responseX[1] + responseY[1] * responseY[1];
  float mag3 = responseX[2] * responseX[2] + responseY[2] * responseY[2];

  if (mag1 > 0.001) proj1 /= mag1;
  if (mag2 > 0.001) proj2 /= mag2;
  if (mag3 > 0.001) proj3 /= mag3;

  // Nudge = GAIN * projection * dt
  // Negative sign: if extending this actuator INCREASES ax, and errX
  // is positive (ax too low vs ref), then we need to extend → negative
  // direction in count space. The response already encodes the sign.
  nudge1 = -GAIN * proj1 * dt;
  nudge2 = -GAIN * proj2 * dt;
  nudge3 = -GAIN * proj3 * dt;

  // Clamp max nudge per step to prevent any sudden jumps
  // At GAIN=0.25, dt=0.012, errX=1500 (25 deg), max nudge would be:
  // 0.06 * 1500/mag * 0.02 ≈ a few counts — already small.
  // But just in case:
  const float MAX_NUDGE = 10.0;  // max counts per loop iteration
  nudge1 = constrain(nudge1, -MAX_NUDGE, MAX_NUDGE);
  nudge2 = constrain(nudge2, -MAX_NUDGE, MAX_NUDGE);
  nudge3 = constrain(nudge3, -MAX_NUDGE, MAX_NUDGE);
}

// ═══════════════════════════════════════════════════════════════════════
// WATCHDOG — detect if system is diverging and reset
// ═══════════════════════════════════════════════════════════════════════

float   prevTotalErr     = 0;
int     divergeCount     = 0;
const int DIVERGE_LIMIT  = 150;  // ~2 seconds of sustained diverging → reset

bool watchdogCheck(float totalErr) {
  if (totalErr > prevTotalErr + 5.0) {
    divergeCount++;
  } else {
    divergeCount = max(divergeCount - 1, 0);
  }
  prevTotalErr = totalErr;

  if (divergeCount >= DIVERGE_LIMIT) {
    Serial.println("!! WATCHDOG: Error diverging — resetting to mid");
    divergeCount = 0;
    return true;  // diverging
  }
  return false;
}

// ═══════════════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════════════

void setup() {
  pinMode(DIR_1, OUTPUT); pinMode(PWM_1, OUTPUT);
  pinMode(DIR_2, OUTPUT); pinMode(PWM_2, OUTPUT);
  pinMode(DIR_3, OUTPUT); pinMode(PWM_3, OUTPUT);

  Wire.begin();
  Serial.begin(9600);
  imu.initialize();
  Serial.println(imu.testConnection() ? "IMU OK" : "!! IMU FAILED — check wiring");

  // ── Step 1: Stiction calibration (individual) ──────
  Serial.println("== STEP 1: Stiction calibration ==");
  STICTION_1 = measureStiction("A1", DIR_1, PWM_1, POT_1);
  STICTION_2 = measureStiction("A2", DIR_2, PWM_2, POT_2);
  STICTION_3 = measureStiction("A3", DIR_3, PWM_3, POT_3);
  Serial.print(">> Stiction: ");
  Serial.print(STICTION_1); Serial.print("/");
  Serial.print(STICTION_2); Serial.print("/");
  Serial.println(STICTION_3);

  // ── Step 2: Go to mid ──────────────────────────────
  Serial.println("== STEP 2: Go to mid ==");
  moveToAndWait(MID_1, MID_2, MID_3);
  Serial.print(">> Mid pots: ");
  Serial.print(analogRead(POT_1)); Serial.print("/");
  Serial.print(analogRead(POT_2)); Serial.print("/");
  Serial.println(analogRead(POT_3));
  delay(500);

  // ── Step 3: Auto-detect IMU-to-actuator mapping ────
  Serial.println("== STEP 3: Direction mapping ==");
  detectDirectionMapping();

  // ── Step 4: Return to mid and lock IMU reference ───
  Serial.println("== STEP 4: Lock IMU reference ==");
  moveToAndWait(MID_1, MID_2, MID_3);
  delay(500);
  lockIMUReference();

  // ── Initialize targets at mid ──────────────────────
  target1 = MID_1;
  target2 = MID_2;
  target3 = MID_3;
  prevTime     = millis();
  lastPrint    = 0;
  correcting   = false;
  divergeCount = 0;
  prevTotalErr = 0;

  Serial.println("================================================");
  Serial.println(">> ACTIVE LEVELING STARTED");
  Serial.println(">> Corrects when > 2 deg off level");
  Serial.println(">> Happy when < 1 deg off level");
  Serial.println(">> Targets walk incrementally — no overshoot");
  Serial.println("================================================");
  Serial.println("deg\tT1\tT2\tT3\tP1\tP2\tP3\tSTATE");
}

// ═══════════════════════════════════════════════════════════════════════
// MAIN LOOP
// ═══════════════════════════════════════════════════════════════════════

void loop() {
  unsigned long now = millis();
  float dt = constrain((now - prevTime) / 1000.0f, 0.001f, 0.1f);
  prevTime = now;

  // ── Read IMU (with spike rejection + low-pass) ─────
  updateIMU();

  // ── Pot sanity check ───────────────────────────────
  int p1 = analogRead(POT_1);
  int p2 = analogRead(POT_2);
  int p3 = analogRead(POT_3);
  if (p1 < 50 || p1 > 710 || p2 < 50 || p2 > 710 || p3 < 50 || p3 > 710) {
    stopAll();
    if (millis() - lastPrint >= 500) {
      lastPrint = millis();
      Serial.println("!! POT FAULT — check wiring");
    }
    delay(100);
    return;
  }

  // ── Compute tilt error ─────────────────────────────
  float errX     = refAx - smoothAx;
  float errY     = refAy - smoothAy;
  float totalErr = sqrt(errX * errX + errY * errY);
  float totalDeg = totalErr / ACCEL_PER_DEG;

  // ── Hysteresis state machine ───────────────────────
  if (!correcting && totalErr > START_DEG * ACCEL_PER_DEG) {
    correcting = true;
    divergeCount = 0;
    prevTotalErr = totalErr;
    Serial.print(">> CORRECTING from "); Serial.print(totalDeg, 1);
    Serial.println(" deg");
  } else if (correcting && totalErr < STOP_DEG * ACCEL_PER_DEG) {
    correcting = false;
    Serial.println(">> HAPPY");
  }

  // ── HAPPY — stop all, hold targets ─────────────────
  if (!correcting) {
    stopAll();

    // While happy, slowly relax targets back toward mid
    // so they don't accumulate drift
    target1 += (MID_1 - target1) * 0.02 * dt;
    target2 += (MID_2 - target2) * 0.02 * dt;
    target3 += (MID_3 - target3) * 0.02 * dt;

    if (millis() - lastPrint >= 500) {
      lastPrint = millis();
      Serial.print(totalDeg, 2); Serial.print("\t");
      Serial.print("---\t---\t---\t");
      Serial.print(p1); Serial.print("\t");
      Serial.print(p2); Serial.print("\t");
      Serial.print(p3); Serial.print("\t");
      Serial.println("HAPPY");
    }
    delay(15);
    return;
  }

  // ── Watchdog — detect divergence ───────────────────
  if (watchdogCheck(totalErr)) {
    stopAll();
    correcting = false;
    target1 = MID_1;
    target2 = MID_2;
    target3 = MID_3;
    smoothAx = refAx;
    smoothAy = refAy;
    moveToAndWait(MID_1, MID_2, MID_3);
    delay(500);
    return;
  }

  // ── Compute incremental nudge for each actuator ────
  float nudge1, nudge2, nudge3;
  computeNudge(errX, errY, dt, nudge1, nudge2, nudge3);

  // ── Update targets incrementally ───────────────────
  target1 += nudge1;
  target2 += nudge2;
  target3 += nudge3;

  // Hard clamp targets to safe range
  target1 = constrain(target1, (float)SAFE_MIN, (float)SAFE_MAX);
  target2 = constrain(target2, (float)SAFE_MIN, (float)SAFE_MAX);
  target3 = constrain(target3, (float)SAFE_MIN, (float)SAFE_MAX);

  // ── Drive actuators toward targets ─────────────────
  driveActuator(DIR_1, PWM_1, POT_1, (int)target1, STICTION_1);
  driveActuator(DIR_2, PWM_2, POT_2, (int)target2, STICTION_2);
  driveActuator(DIR_3, PWM_3, POT_3, (int)target3, STICTION_3);

  // ── Print status ───────────────────────────────────
  if (millis() - lastPrint >= 200) {
    lastPrint = millis();
    Serial.print(totalDeg, 2);    Serial.print("\t");
    Serial.print((int)target1);   Serial.print("\t");
    Serial.print((int)target2);   Serial.print("\t");
    Serial.print((int)target3);   Serial.print("\t");
    Serial.print(p1);             Serial.print("\t");
    Serial.print(p2);             Serial.print("\t");
    Serial.print(p3);             Serial.print("\t");
    Serial.println("CORRECTING");
  }

  delay(12);  // ~80Hz loop
}
