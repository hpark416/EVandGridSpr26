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

// ---------- Brake light source (change here) ----------
enum BrakeLightMode : uint8_t {
  BRAKE_LIGHT_IMU_ONLY = 0,
  BRAKE_LIGHT_IMU_AND_RC = 1,
};

static const BrakeLightMode BRAKE_LIGHT_MODE = BRAKE_LIGHT_IMU_AND_RC;

static const bool RC_BRAKE_ON_NEGATIVE_THROTTLE = true;

static const float RC_BRAKE_CURVE_EXP = 1.35f;

static const float RC_BRAKE_ATTACK_ALPHA  = 0.55f;
static const float RC_BRAKE_RELEASE_ALPHA = 0.16f;

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

const int rxDeadband = 30;

const unsigned long pulseTimeout = 30000UL;

const uint32_t RC_MS = 10;

uint32_t lastRc = 0;

static float readAxis(int pin, bool invert = false) {
  unsigned long pulse = pulseIn(pin, HIGH, pulseTimeout);

  if (pulse < 900 || pulse > 2100) {
    return 0.0f;
  }

  int centered = (int)pulse - rxMid;

  if (abs(centered) <= rxDeadband) {
    return 0.0f;
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

  return value;
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

// Built-in LED (classic Nano): rough status — NeoPixels carry main brake UI
const int STATUS_LED_PIN = LED_BUILTIN;

// 0 = X, 1 = Y, 2 = Z (adjust if MPU6050 mount differs from vehicle forward)
#define FORWARD_AXIS 0

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
const float AX_LP_ALPHA = 0.02f;

const float BRAKE_START_G      = 0.09f;
const float BRAKE_FULL_G       = 0.28f;
const float BRAKE_OFF_HYST_G   = 0.03f;
const float BRAKE_SMOOTH_ALPHA = 0.25f;

const bool HARD_FLASH_ENABLE = true;
const float HARD_FLASH_G = 0.34f;
const uint32_t HARD_FLASH_MS = 120;

uint32_t lastImu = 0, lastLed = 0;
float motionMetricG = 0.0f;
bool moving = false;

bool brakeArmed = false;
float brakeLevelImu = 0.0f;
float brakeLevelRc = 0.0f;
uint32_t flashUntil = 0;

static float rcBrakeTargetFromThrottle(float y) {
  float pull = RC_BRAKE_ON_NEGATIVE_THROTTLE ? (-y) : (y);
  if (pull <= 0.0f) return 0.0f;
  pull = clamp01(pull);
  if (RC_BRAKE_CURVE_EXP != 1.0f) {
    pull = powf(pull, RC_BRAKE_CURVE_EXP);
  }
  return clamp01(pull);
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

static float combinedBrakeBarLevel() {
  if (BRAKE_LIGHT_MODE == BRAKE_LIGHT_IMU_ONLY) {
    return brakeLevelImu;
  }
  return fmaxf(brakeLevelImu, brakeLevelRc);
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
  Serial.println(BRAKE_LIGHT_MODE == BRAKE_LIGHT_IMU_ONLY ? F("IMU_ONLY") : F("IMU_AND_RC"));

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

    float aFwd = pickForwardAxis(ax, ay, az);
    aFwdLP = (1.0f - AX_LP_ALPHA) * aFwdLP + AX_LP_ALPHA * aFwd;
    float aFwdHP = aFwd - aFwdLP;

    float brakeG = fabsf(aFwdHP);

    if (!brakeArmed) {
      if (brakeG > BRAKE_START_G) brakeArmed = true;
    } else {
      if (brakeG < (BRAKE_START_G - BRAKE_OFF_HYST_G)) brakeArmed = false;
    }

    float target = 0.0f;
    if (brakeArmed) {
      target = (brakeG - BRAKE_START_G) / (BRAKE_FULL_G - BRAKE_START_G);
      target = clamp01(target);
    }

    brakeLevelImu = (1.0f - BRAKE_SMOOTH_ALPHA) * brakeLevelImu + BRAKE_SMOOTH_ALPHA * target;

    if (HARD_FLASH_ENABLE && brakeG > HARD_FLASH_G) {
      flashUntil = now + HARD_FLASH_MS;
    }
  }

  if (now - lastLed >= LED_MS) {
    lastLed = now;

    const bool hardFlash = (now < flashUntil);
    const float barLevel = combinedBrakeBarLevel();

    if (hardFlash) {
      setBrakeBar(1.0f, true);
    } else {
      setBrakeBar(barLevel, false);
    }

    updateStatusLed(now, hardFlash, barLevel, moving);
  }

  if (now - lastRc >= RC_MS) {
    lastRc = now;

    float x = readAxis(rxXPin, false);
    float y = readAxis(rxYPin, false);

    float left  = y + x;
    float right = y - x;

    float maxMag = max(1.0f, max(fabsf(left), fabsf(right)));
    left  /= maxMag;
    right /= maxMag;

    const int pwmL = axisToPWM(left);
    const int pwmR = axisToPWM(right);
    analogWrite(pwmPinLeft, pwmL);
    analogWrite(pwmPinRight, pwmR);

    if (BRAKE_LIGHT_MODE == BRAKE_LIGHT_IMU_AND_RC) {
      smoothRcBrakeToward(rcBrakeTargetFromThrottle(y));
    } else {
      brakeLevelRc = 0.0f;
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
    Serial.println(combinedBrakeBarLevel(), 2);
  }
}
