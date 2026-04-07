#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_NeoPixel.h>
#include <math.h>

// =============================================================================
// RC: RadioLink R8EF -> Arduino Nano (5 V) -> MCP602 buffered motor PWM
//     D2 = X / steering, D3 = Y / throttle
//     D9 = left motor, D10 = right motor (arcade / tank mix)
// Brake: NeoPixel strip on D6; IMU: external MPU6050 on I2C (A4/A5), addr 0x68
// Status: built-in LED D13 (no RGB like Nano 33 BLE)
// =============================================================================

// ---------- Brake light source (pick ONE — mutually exclusive) ----------
// Brake *bar level* is either from RC transmitter logic OR from IMU physics, never both.
// Set BRAKE_LIGHT_MODE below; recompile and upload to switch.
enum BrakeLightMode : uint8_t {
  BRAKE_LIGHT_IMU_ONLY = 0,   // brake bar from MPU6050 deceleration only
  BRAKE_LIGHT_RC_ONLY = 1,    // brake bar from RC only (stick + lift-off); IMU not used for bar
};

static const BrakeLightMode BRAKE_LIGHT_MODE = 0;

// Set true if "forward throttle" reads positive on your receiver channel,
// false if "forward throttle" reads negative.
static const bool RC_BRAKE_ON_NEGATIVE_THROTTLE = false;

static const float RC_BRAKE_CURVE_EXP = 1.35f;

static const float RC_BRAKE_ATTACK_ALPHA  = 0.55f;
static const float RC_BRAKE_RELEASE_ALPHA = 0.16f;

// Regen-like brake cue from throttle lift-off speed (per second in normalized units).
const float RC_LIFTOFF_START_PER_S = 1.00f;
const float RC_LIFTOFF_FULL_PER_S  = 2.40f;
// Rapid lift-off only: must exceed mild/moderate lift-off (see RC_LIFTOFF_*).
const float RC_RAPID_DROP_PER_S = 5.50f;
const uint32_t RC_LIFTOFF_BLINK_MS = 480;

// ---------- RC pins & calibration ----------
const int rxXPin = 2;
const int rxYPin = 3;

const int pwmPinLeft  = 9;
const int pwmPinRight = 10;

const int minPWM     = 83;   // ~1.6 V
const int neutralPWM = 140;  // ~2.7 V
const int maxPWM     = 193;  // ~3.7 V

const int pwmRangeUp   = maxPWM - neutralPWM;
const int pwmRangeDown = neutralPWM - minPWM;

const int rxMin = 1000;
const int rxMid = 1500;
const int rxMax = 2000;

const int rxDeadband = 0;

const unsigned long pulseTimeout = 30000UL;

const uint32_t RC_MS = 10;

uint32_t lastRc = 0;

static bool readAxis(int pin, float &valueOut, bool invert = false) {
  unsigned long pulse = pulseIn(pin, HIGH, pulseTimeout);

  if (pulse < 900 || pulse > 2100) {
    valueOut = 0.0f;
    return false;
  }

  int centered = (int)pulse - rxMid;

  if (abs(centered) <= rxDeadband) {
    valueOut = 0.0f;
    return true;
  }

  float value;
  if (centered > 0) {
    value = (float)centered / (float)(rxMax - rxMid);
  } else {
    value = (float)centered / (float)(rxMid - rxMin);
  }

  value = constrain(value, -1.0f, 1.0f);

  if (invert) {
    value = -value;
  }

  valueOut = value;
  return true;
}

static int axisToPWM(float value) {
  value = constrain(value, -1.0f, 1.0f);

  if (value >= 0.0f) {
    return neutralPWM + (int)(value * (float)pwmRangeUp);
  }
  return neutralPWM + (int)(value * (float)pwmRangeDown);
}

// ---------- MPU6050 ----------
Adafruit_MPU6050 mpu;

static const float GRAVITY_MS2 = 9.80665f;

// ---------- NeoPixel ----------
#define LED_PIN     6
#define NUMPIXELS   8
Adafruit_NeoPixel strip(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

const uint8_t PIXEL_BRIGHTNESS_CAP = 40;

const uint8_t TAIL_R_BASE = 18;
const uint8_t BRAKE_R_MAX = 255;
const uint8_t REVERSE_WHITE_MAX = 210;
const uint8_t EV_CRUISE_G_MAX = 180;
const uint8_t EV_CRUISE_B_MAX = 255;

const float DRIVE_SMOOTH_ALPHA = 0.22f;
// Display-only neutral hysteresis to suppress LED chatter near stick center.
const float DISPLAY_DRIVE_ENTER = 0.10f;
const float DISPLAY_DRIVE_EXIT  = 0.07f;
const float BRAKE_DISPLAY_LEVEL_NEUTRAL = 0.01f;
const float BRAKE_DISPLAY_LEVEL_FORWARD = 0.22f;
const float EV_SPEED_SMOOTH_ALPHA = 0.20f;
// Regen-to-red tuning for forward animation.
const float EV_REGEN_RED_START = 0.18f;   // brake level before red tint begins
const float EV_REGEN_RED_CURVE = 1.80f;   // >1 = slower early rise, faster near strong braking
const float EV_REGEN_SMOOTH_ALPHA = 0.18f;

// RC-only: amber/yellow turn hint on outer pixels (blinking, scales with |steer|).
const float TURN_STEER_DEADBAND = 0.06f;
const uint8_t TURN_MAX_PIXELS_PER_SIDE = 3;
const uint8_t TURN_AMBER_R = 220;
const uint8_t TURN_AMBER_G = 140;
const uint32_t TURN_BLINK_PERIOD_MS = 115;

// Built-in LED (classic Nano): rough status — NeoPixels carry main brake UI
const int STATUS_LED_PIN = LED_BUILTIN;

// MPU6050 on Segway-style kit (e.g. Meka): mount with board flat, X = forward travel,
// Z = vertical; FORWARD_AXIS 0 = longitudinal braking on accel X.
// 0 = X, 1 = Y, 2 = Z (change if your bracket differs)
#define FORWARD_AXIS 0
// +1 or -1: flip if brake lights trigger when accelerating (wrong axis sign).
#define FORWARD_AXIS_SIGN 1

// Only use IMU brake estimate when motion detector says we are moving (reduces idle noise).
static const bool IMU_BRAKE_REQUIRE_MOVING = true;

// Set true while tuning IMU by moving/shaking the board in your hand: higher gain, lower
// thresholds, ignores "moving" gate so decel is obvious. Keep false on the vehicle.
static const bool IMU_HAND_TUNE = false;
static const float IMU_HAND_GAIN = 2.4f;  // multiplies decel signal before smoothing

static inline float pickForwardAxis(float ax, float ay, float az) {
#if FORWARD_AXIS == 0
  return ax;
#elif FORWARD_AXIS == 1
  return ay;
#else
  return az;
#endif
}

static inline float clamp01(float v) {
  if (v < 0.0f) return 0.0f;
  if (v > 1.0f) return 1.0f;
  return v;
}

// D13: on = braking / hard flash; slow blink = moving; off = idle
static void updateStatusLed(uint32_t now, bool hardFlash, float barLevel, bool movingActive) {
  if (hardFlash) {
    digitalWrite(STATUS_LED_PIN, HIGH);
    return;
  }
  if (barLevel > 0.05f) {
    digitalWrite(STATUS_LED_PIN, HIGH);
    return;
  }
  if (movingActive) {
    digitalWrite(STATUS_LED_PIN, (now / 250UL) & 1UL ? HIGH : LOW);
    return;
  }
  digitalWrite(STATUS_LED_PIN, LOW);
}

// ---------- Brake timing & tuning (HP IMU path, g) ----------
const uint32_t IMU_MS = 5;
const uint32_t LED_MS = 20;

const float MOTION_ON_G  = 0.10f;
const float MOTION_OFF_G = 0.05f;
const float MOTION_ALPHA = 0.30f;

float aFwdLP = 0.0f;
// Slightly faster LP tracks gravity on the forward axis a bit better when the body pitches.
const float AX_LP_ALPHA = 0.035f;

const float BRAKE_START_G      = 0.09f;
const float BRAKE_FULL_G       = 0.28f;
const float BRAKE_OFF_HYST_G   = 0.03f;
const float BRAKE_SMOOTH_ALPHA = 0.25f;
// Extra smoothing on *deceleration-only* signal before thresholds (reduces spike false triggers).
const float BRAKE_DECEL_SMOOTH_ALPHA = 0.40f;

const bool HARD_FLASH_ENABLE = true;
const float HARD_FLASH_G = 0.34f;
const uint32_t HARD_FLASH_MS = 120;

// IMU-only: match RC visuals — cruise speed from motion; rapid decel blink threshold (g-equivalent HP).
const float IMU_CRUISE_SPEED_ALPHA = 0.16f;
const float IMU_RAPID_DECEL_BLINK_G = 0.26f;
const float IMU_VIS_FORWARD_BRAKE_MAX = 0.42f;

uint32_t lastImu = 0, lastLed = 0;
float motionMetricG = 0.0f;
bool moving = false;

bool brakeArmed = false;
float brakeLevelImu = 0.0f;
float brakeDecelGSmooth = 0.0f;
float brakeLevelRc = 0.0f;
uint32_t flashUntil = 0;
float prevForwardThrottle = 0.0f;
float rcForwardLevel = 0.0f;
float rcReverseLevel = 0.0f;
float rcDriveLevel = 0.0f;
int8_t displayDriveState = 0; // -1 reverse, 0 neutral, +1 forward
float evCruiseHeadPos = 0.0f;
float evCruiseBreathPhase = 0.0f;
float evCruiseSpeedSmoothed = 0.0f;
float evCruiseRegenSmoothed = 0.0f;
uint32_t rcLiftOffBlinkUntil = 0;
uint32_t imuRapidBlinkUntil = 0;
float imuCruiseSpeed01 = 0.0f;
float rcSteerX = 0.0f;

static float rcBrakeTargetFromThrottle(float y) {
  float pull = RC_BRAKE_ON_NEGATIVE_THROTTLE ? (-y) : (y);
  if (pull <= 0.0f) return 0.0f;
  pull = clamp01(pull);
  if (RC_BRAKE_CURVE_EXP != 1.0f) {
    pull = powf(pull, RC_BRAKE_CURVE_EXP);
  }
  return clamp01(pull);
}

static float rcForwardThrottle(float y) {
  float fwd = RC_BRAKE_ON_NEGATIVE_THROTTLE ? y : (-y);
  return clamp01(fwd);
}

static float rcSignedDrive(float y) {
  const float v = RC_BRAKE_ON_NEGATIVE_THROTTLE ? y : (-y);
  return constrain(v, -1.0f, 1.0f);
}

static void smoothRcBrakeToward(float target) {
  const float a = (target > brakeLevelRc) ? RC_BRAKE_ATTACK_ALPHA : RC_BRAKE_RELEASE_ALPHA;
  brakeLevelRc += a * (target - brakeLevelRc);
  if (brakeLevelRc < 0.001f) brakeLevelRc = 0.0f;
  else if (brakeLevelRc > 0.999f) brakeLevelRc = 1.0f;
}

const uint8_t CENTER_OUT[NUMPIXELS] = {3, 4, 2, 5, 1, 6, 0, 7};

static void setTailAll() {
  for (int i = 0; i < NUMPIXELS; i++) {
    strip.setPixelColor(i, strip.Color(TAIL_R_BASE, 0, 0));
  }
}

static void setAllColor(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < NUMPIXELS; i++) {
    strip.setPixelColor(i, strip.Color(r, g, b));
  }
  strip.show();
}

static void setEvCruise(float speed01, float regen01 = 0.0f) {
  speed01 = clamp01(speed01);
  regen01 = clamp01(regen01);
  evCruiseSpeedSmoothed += EV_SPEED_SMOOTH_ALPHA * (speed01 - evCruiseSpeedSmoothed);
  const float speedSmoothed = clamp01(evCruiseSpeedSmoothed);
  evCruiseRegenSmoothed += EV_REGEN_SMOOTH_ALPHA * (regen01 - evCruiseRegenSmoothed);
  const float regenSmoothed = clamp01(evCruiseRegenSmoothed);
  const float regenNorm = clamp01((regenSmoothed - EV_REGEN_RED_START) / (1.0f - EV_REGEN_RED_START));
  const float regenBlend = powf(regenNorm, EV_REGEN_RED_CURVE);

  const float dt = (float)LED_MS / 1000.0f;
  const float scanHz = 0.30f + 3.00f * speedSmoothed;
  evCruiseHeadPos += scanHz * (float)NUMPIXELS * dt;
  while (evCruiseHeadPos >= (float)NUMPIXELS) evCruiseHeadPos -= (float)NUMPIXELS;

  const float breathHz = 0.30f + speedSmoothed;
  evCruiseBreathPhase += 2.0f * PI * breathHz * dt;
  while (evCruiseBreathPhase >= 2.0f * PI) evCruiseBreathPhase -= 2.0f * PI;

  const float breathe = 0.60f + 0.40f * (0.5f + 0.5f * sinf(evCruiseBreathPhase));

  for (int i = 0; i < NUMPIXELS; i++) {
    const float d = fabsf((float)i - evCruiseHeadPos);
    const float wrapD = fminf(d, (float)NUMPIXELS - d);
    const float trail = clamp01(1.0f - wrapD / 3.0f);

    const float tealScale = 1.0f - 0.85f * regenBlend;
    const float rBase = regenBlend * (12.0f + speedSmoothed * 55.0f);
    const float rPulse = trail * (60.0f + regenBlend * 190.0f) * breathe;
    const float gBase = (10.0f + speedSmoothed * 40.0f) * tealScale;
    const float bBase = (18.0f + speedSmoothed * 55.0f) * tealScale;
    const float gPulse = trail * (float)EV_CRUISE_G_MAX * breathe * tealScale;
    const float bPulse = trail * (float)EV_CRUISE_B_MAX * breathe * tealScale;

    const uint8_t r = (uint8_t)constrain((int)(rBase + rPulse), 0, 255);
    const uint8_t g = (uint8_t)constrain((int)(gBase + gPulse), 0, 255);
    const uint8_t b = (uint8_t)constrain((int)(bBase + bPulse), 0, 255);
    strip.setPixelColor(i, strip.Color(r, g, b));
  }
  strip.show();
}

static void setReverseWhite(uint32_t now, float reverse01) {
  reverse01 = clamp01(reverse01);
  const float t = (float)now / 1000.0f;
  const float pulse = 0.5f + 0.5f * sinf(2.0f * PI * 0.85f * t);
  const float wLo = 22.0f + reverse01 * 30.0f;
  const float wHi = 55.0f + reverse01 * (float)(REVERSE_WHITE_MAX - 55);
  const uint8_t w = (uint8_t)(wLo + (wHi - wLo) * pulse);
  setAllColor(w, w, w);
}

static void setBrakeBar(float level01, bool hardFlash) {
  setTailAll();

  int n = (int)(level01 * (float)NUMPIXELS + 0.5f);
  if (n < 0) n = 0;
  if (n > NUMPIXELS) n = NUMPIXELS;

  uint8_t brakeR = hardFlash ? BRAKE_R_MAX : (uint8_t)(60 + level01 * 195);

  for (int k = 0; k < n; k++) {
    uint8_t idx = CENTER_OUT[k];
    strip.setPixelColor(idx, strip.Color(brakeR, 0, 0));
  }

  strip.show();
}

// Full strip, uniform brake red (no center-out partial fill) — use when stick has no direction.
static void setBrakeSolidRed() {
  const uint8_t r = BRAKE_R_MAX;
  for (int i = 0; i < NUMPIXELS; i++) {
    strip.setPixelColor(i, strip.Color(r, 0, 0));
  }
  strip.show();
}

// Blinking red bar (rapid lift-off warning).
static void setBrakeBarBlink(uint32_t now, float level01) {
  level01 = clamp01(level01);
  const bool on = ((now / 90UL) & 1UL) != 0;
  if (on) {
    setBrakeBar(level01, false);
  } else {
    setTailAll();
    strip.show();
  }
}

// RC + IMU modes: blink amber on outer strip ends from steering (replace, not additive).
static void applyTurnOverlay(uint32_t now) {
  if (BRAKE_LIGHT_MODE != BRAKE_LIGHT_RC_ONLY && BRAKE_LIGHT_MODE != BRAKE_LIGHT_IMU_ONLY) {
    return;
  }
  const float ax = fabsf(rcSteerX);
  if (ax <= TURN_STEER_DEADBAND) {
    return;
  }
  const float turn01 = clamp01((ax - TURN_STEER_DEADBAND) / (1.0f - TURN_STEER_DEADBAND));

  int nSide = 1 + (int)((float)(TURN_MAX_PIXELS_PER_SIDE - 1) * turn01 + 0.5f);
  if (nSide < 1) nSide = 1;
  if (nSide > (int)TURN_MAX_PIXELS_PER_SIDE) nSide = (int)TURN_MAX_PIXELS_PER_SIDE;

  static uint32_t stripBeforeTurn[NUMPIXELS];
  for (int i = 0; i < NUMPIXELS; i++) {
    stripBeforeTurn[i] = strip.getPixelColor((uint16_t)i);
  }

  const bool turnBlinkOn = ((now / TURN_BLINK_PERIOD_MS) & 1UL) != 0;

  for (int k = 0; k < nSide; k++) {
    const int i = (rcSteerX < 0.0f) ? k : (NUMPIXELS - 1 - k);
    if (i < 0 || i >= NUMPIXELS) continue;
    if (turnBlinkOn) {
      strip.setPixelColor((uint16_t)i, TURN_AMBER_R, TURN_AMBER_G, 0);
    } else {
      strip.setPixelColor((uint16_t)i, stripBeforeTurn[(uint16_t)i]);
    }
  }
  strip.show();
}

// Bar level for NeoPixels and status.
static float displayBarLevel() {
  if (BRAKE_LIGHT_MODE == BRAKE_LIGHT_IMU_ONLY) {
    return brakeLevelImu;
  }
  return brakeLevelRc;
}

void setup() {
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  pinMode(rxXPin, INPUT);
  pinMode(rxYPin, INPUT);
  pinMode(pwmPinLeft, OUTPUT);
  pinMode(pwmPinRight, OUTPUT);
  analogWrite(pwmPinLeft, neutralPWM);
  analogWrite(pwmPinRight, neutralPWM);

  strip.begin();
  strip.setBrightness(PIXEL_BRIGHTNESS_CAP);
  strip.show();

  Serial.begin(115200);
  Serial.print(F("RcAndBrakeLights (Nano+MPU6050): brake mode "));
  Serial.println(BRAKE_LIGHT_MODE == BRAKE_LIGHT_IMU_ONLY ? F("IMU_ONLY") : F("RC_ONLY"));
  if (IMU_HAND_TUNE) {
    Serial.println(F("IMU_HAND_TUNE=1 (sensitive; set false on vehicle)"));
  }

  Wire.begin();

  if (!mpu.begin()) {
    Serial.println(F("MPU6050 not found (check A4/A5, 0x68, power)."));
    while (1) {
      digitalWrite(STATUS_LED_PIN, HIGH);
      delay(100);
      digitalWrite(STATUS_LED_PIN, LOW);
      delay(100);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroStandby(true, true, true);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  setTailAll();
  strip.show();
  digitalWrite(STATUS_LED_PIN, LOW);

  lastRc = lastImu = lastLed = millis();
}

void loop() {
  const uint32_t now = millis();

  if (now - lastImu >= IMU_MS) {
    lastImu = now;

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float ax = a.acceleration.x / GRAVITY_MS2;
    float ay = a.acceleration.y / GRAVITY_MS2;
    float az = a.acceleration.z / GRAVITY_MS2;

    float amag = sqrtf(ax * ax + ay * ay + az * az);
    float dev  = fabsf(amag - 1.0f);
    motionMetricG = (1.0f - MOTION_ALPHA) * motionMetricG + MOTION_ALPHA * dev;

    if (!moving && motionMetricG > MOTION_ON_G) moving = true;
    if ( moving && motionMetricG < MOTION_OFF_G) moving = false;

    float aFwd = pickForwardAxis(ax, ay, az) * (float)FORWARD_AXIS_SIGN;
    aFwdLP = (1.0f - AX_LP_ALPHA) * aFwdLP + AX_LP_ALPHA * aFwd;
    float aFwdHP = aFwd - aFwdLP;

    // Deceleration only: positive when longitudinal accel drops (braking), not when accelerating.
    // Using fabs(high-pass) also lit the bar on accel spikes and bumps.
    float brakeG = fmaxf(0.0f, -aFwdHP);
    if (IMU_HAND_TUNE) {
      brakeG *= IMU_HAND_GAIN;
    }
    brakeDecelGSmooth += BRAKE_DECEL_SMOOTH_ALPHA * (brakeG - brakeDecelGSmooth);

    const float startG = IMU_HAND_TUNE ? 0.035f : BRAKE_START_G;
    const float fullG  = IMU_HAND_TUNE ? 0.16f : BRAKE_FULL_G;
    const float hardG  = IMU_HAND_TUNE ? 0.15f : HARD_FLASH_G;

    const bool imuOk =
        (!IMU_BRAKE_REQUIRE_MOVING) || moving || IMU_HAND_TUNE;
    const float brakeUse = imuOk ? brakeDecelGSmooth : 0.0f;

    if (!brakeArmed) {
      if (brakeUse > startG) brakeArmed = true;
    } else {
      if (brakeUse < (startG - BRAKE_OFF_HYST_G)) brakeArmed = false;
    }

    float target = 0.0f;
    if (brakeArmed) {
      target = (brakeUse - startG) / (fullG - startG);
      target = clamp01(target);
    }

    brakeLevelImu = (1.0f - BRAKE_SMOOTH_ALPHA) * brakeLevelImu + BRAKE_SMOOTH_ALPHA * target;

    if (HARD_FLASH_ENABLE && brakeUse > hardG) {
      flashUntil = now + HARD_FLASH_MS;
    }

    if (BRAKE_LIGHT_MODE == BRAKE_LIGHT_IMU_ONLY) {
      const float cruiseTarget =
          clamp01((motionMetricG - MOTION_OFF_G * 0.85f) / fmaxf(MOTION_ON_G * 2.2f, 0.001f));
      imuCruiseSpeed01 += IMU_CRUISE_SPEED_ALPHA * (cruiseTarget - imuCruiseSpeed01);
      if (brakeG > IMU_RAPID_DECEL_BLINK_G) {
        imuRapidBlinkUntil = now + RC_LIFTOFF_BLINK_MS;
      }
    }
  }

  if (now - lastLed >= LED_MS) {
    lastLed = now;

    const bool imuFlashPending = (now < flashUntil);
    // Strip: IMU flash only in IMU-only mode (RC-only bar stays TX-driven).
    const bool hardFlashStrip = imuFlashPending && (BRAKE_LIGHT_MODE != BRAKE_LIGHT_RC_ONLY);
    if (displayDriveState == 0) {
      if (rcDriveLevel > DISPLAY_DRIVE_ENTER) displayDriveState = 1;
      else if (rcDriveLevel < -DISPLAY_DRIVE_ENTER) displayDriveState = -1;
    } else if (displayDriveState > 0) {
      if (rcDriveLevel < DISPLAY_DRIVE_EXIT) displayDriveState = 0;
    } else {
      if (rcDriveLevel > -DISPLAY_DRIVE_EXIT) displayDriveState = 0;
    }

    const bool reverseActive = (displayDriveState < 0);
    const bool forwardImuScooter =
        (BRAKE_LIGHT_MODE == BRAKE_LIGHT_IMU_ONLY) && !reverseActive && moving &&
        (brakeLevelImu < IMU_VIS_FORWARD_BRAKE_MAX) && (motionMetricG > MOTION_OFF_G * 1.1f);
    const bool forwardActive = (displayDriveState > 0) || forwardImuScooter;
    const float barLevel = displayBarLevel();
    const bool liftOffBlink =
        ((now < rcLiftOffBlinkUntil) || (now < imuRapidBlinkUntil)) && !reverseActive;
    const float brakeDisplayThreshold = forwardActive ? BRAKE_DISPLAY_LEVEL_FORWARD : BRAKE_DISPLAY_LEVEL_NEUTRAL;
    const bool brakeActive = (barLevel > brakeDisplayThreshold);

    if (hardFlashStrip) {
      setBrakeBar(1.0f, true);
    } else if (reverseActive) {
      setReverseWhite(now, rcReverseLevel);
    } else if (liftOffBlink) {
      if (displayDriveState == 0) {
        setBrakeBarBlink(now, 1.0f);
      } else {
        setBrakeBarBlink(now, fmaxf(barLevel, 0.85f));
      }
    } else if (forwardActive) {
      float cruiseSpeed = rcForwardLevel;
      if (BRAKE_LIGHT_MODE == BRAKE_LIGHT_IMU_ONLY) {
        cruiseSpeed = fmaxf(rcForwardLevel, imuCruiseSpeed01);
      }
      setEvCruise(cruiseSpeed, barLevel);
    } else if (brakeActive) {
      if (displayDriveState == 0) {
        setBrakeSolidRed();
      } else {
        setBrakeBar(barLevel, false);
      }
    } else {
      setTailAll();
      strip.show();
    }

    if (BRAKE_LIGHT_MODE == BRAKE_LIGHT_RC_ONLY || BRAKE_LIGHT_MODE == BRAKE_LIGHT_IMU_ONLY) {
      applyTurnOverlay(now);
    }

    updateStatusLed(now, imuFlashPending, barLevel, moving);
  }

  if (now - lastRc >= RC_MS) {
    lastRc = now;

    float x = 0.0f, y = 0.0f;
    const bool xValid = readAxis(rxXPin, x, false);
    const bool yValid = readAxis(rxYPin, y, false);
    const bool rcSignalOk = xValid && yValid;

    if (!rcSignalOk) {
      // Fail-safe behavior: no valid RC pulses -> neutral motor + full brake bar.
      analogWrite(pwmPinLeft, neutralPWM);
      analogWrite(pwmPinRight, neutralPWM);
      rcForwardLevel = 0.0f;
      rcReverseLevel = 0.0f;
      rcDriveLevel = 0.0f;
      displayDriveState = 0;
      evCruiseSpeedSmoothed = 0.0f;
      evCruiseRegenSmoothed = 0.0f;
      rcLiftOffBlinkUntil = 0;
      imuRapidBlinkUntil = 0;
      imuCruiseSpeed01 = 0.0f;
      if (BRAKE_LIGHT_MODE == BRAKE_LIGHT_IMU_ONLY) {
        brakeLevelRc = 0.0f;
      } else {
        brakeLevelRc = 1.0f;
      }
      prevForwardThrottle = 0.0f;
      rcSteerX = 0.0f;

      Serial.println(F("RC signal lost: neutral motor, full brake bar."));
      return;
    }

    rcSteerX = x;

    float left  = y + x;
    float right = y - x;

    float maxMag = max(1.0f, max(fabsf(left), fabsf(right)));
    left  /= maxMag;
    right /= maxMag;

    const int pwmL = axisToPWM(left);
    const int pwmR = axisToPWM(right);
    analogWrite(pwmPinLeft, pwmL);
    analogWrite(pwmPinRight, pwmR);

    rcForwardLevel = rcForwardThrottle(y);
    rcReverseLevel = RC_BRAKE_ON_NEGATIVE_THROTTLE ? clamp01(-y) : clamp01(y);
    const float driveRaw = rcSignedDrive(y);
    rcDriveLevel += DRIVE_SMOOTH_ALPHA * (driveRaw - rcDriveLevel);
    if (fabsf(rcDriveLevel) < 0.001f) rcDriveLevel = 0.0f;

    const float fwdNow = rcForwardThrottle(y);
    const float drop = prevForwardThrottle - fwdNow;
    prevForwardThrottle = fwdNow;
    float brakeFromLift = 0.0f;
    if (drop > 0.0f) {
      const float dropPerSec = drop * (1000.0f / (float)RC_MS);
      if (dropPerSec > RC_LIFTOFF_START_PER_S) {
        brakeFromLift = clamp01((dropPerSec - RC_LIFTOFF_START_PER_S) /
                                (RC_LIFTOFF_FULL_PER_S - RC_LIFTOFF_START_PER_S));
      }
      if (dropPerSec >= RC_RAPID_DROP_PER_S) {
        rcLiftOffBlinkUntil = now + RC_LIFTOFF_BLINK_MS;
      }
    }

    if (BRAKE_LIGHT_MODE == BRAKE_LIGHT_IMU_ONLY) {
      brakeLevelRc = 0.0f;
    } else {
      const float brakeFromStick = rcBrakeTargetFromThrottle(y);
      smoothRcBrakeToward(fmaxf(brakeFromStick, brakeFromLift));
    }

    Serial.print(F("X="));
    Serial.print(x, 3);
    Serial.print(F(" Y="));
    Serial.print(y, 3);
    Serial.print(F(" | L="));
    Serial.print(pwmL);
    Serial.print(F(" R="));
    Serial.print(pwmR);
    Serial.print(F(" | Bimu="));
    Serial.print(brakeLevelImu, 2);
    Serial.print(F(" Brc="));
    Serial.print(brakeLevelRc, 2);
    Serial.print(F(" Bbar="));
    Serial.println(displayBarLevel(), 2);
  }
}
