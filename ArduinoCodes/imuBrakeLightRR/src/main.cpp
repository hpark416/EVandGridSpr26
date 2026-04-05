#include <Arduino.h>
#include <Arduino_LSM9DS1.h>
#include <Adafruit_NeoPixel.h>
#include <math.h>

// ---------- NeoPixel ----------
#define LED_PIN     6
#define NUMPIXELS   8
Adafruit_NeoPixel strip(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

// Keep low for 3.3V-pin power
const uint8_t PIXEL_BRIGHTNESS_CAP = 40;

// Tail and brake brightness
const uint8_t TAIL_R_BASE = 18;     // dim red tail
const uint8_t BRAKE_R_MAX = 255;    // full red (scaled by brightness cap)

// ---------- Onboard RGB LED (active LOW) ----------
#ifndef LEDR
  #define LEDR 22
  #define LEDG 23
  #define LEDB 24
#endif

// 0 = X, 1 = Y, 2 = Z  (set forward axis for best behavior)
#define FORWARD_AXIS 0

// If braking lights during hard acceleration, flip this sign:
//  +1 = braking corresponds to negative forward accel (typical)
//  -1 = braking corresponds to positive forward accel (axis reversed)
#define BRAKE_SIGN +1

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

// Onboard RGB: active-low PWM (0=ON, 255=OFF)
void onboardRGB(uint8_t r, uint8_t g, uint8_t b) {
  analogWrite(LEDR, r);
  analogWrite(LEDG, g);
  analogWrite(LEDB, b);
}

// ---------- Timing ----------
const uint32_t IMU_MS = 5;
const uint32_t LED_MS = 20;

// ---------- Motion detect ----------
const float MOTION_ON_G  = 0.10f;
const float MOTION_OFF_G = 0.05f;
const float MOTION_ALPHA = 0.30f;

// ---------- Forward accel filtering ----------
float aFwdLP = 0.0f;
const float AX_LP_ALPHA = 0.02f;

// ---------- Brake intensity (HARD DECEL ONLY) ----------
const float BRAKE_START_G      = 0.09f;  // below this => no brake
const float BRAKE_FULL_G       = 0.28f;  // at/above this => full brake bar
const float BRAKE_OFF_HYST_G   = 0.03f;
const float BRAKE_SMOOTH_ALPHA = 0.25f;

// Optional hard-brake flash
const bool HARD_FLASH_ENABLE = true;
const float HARD_FLASH_G = 0.34f;
const uint32_t HARD_FLASH_MS = 120;

// ---------- State ----------
uint32_t lastImu = 0, lastLed = 0;
float motionMetricG = 0.0f;
bool moving = false;

bool brakeArmed = false;
float brakeLevel = 0.0f; // 0..1
uint32_t flashUntil = 0;

// Center-out pixel order for 8 LEDs:
// indices: 3,4,2,5,1,6,0,7
const uint8_t CENTER_OUT[NUMPIXELS] = {3, 4, 2, 5, 1, 6, 0, 7};

void setTailAll() {
  for (int i = 0; i < NUMPIXELS; i++) {
    strip.setPixelColor(i, strip.Color(TAIL_R_BASE, 0, 0));
  }
}

void setBrakeBar(float level01, bool hardFlash) {
  // Always start from tail
  setTailAll();

  // How many pixels should be "brake lit" (0..8)
  int n = (int)(level01 * (float)NUMPIXELS + 0.5f);
  if (n < 0) n = 0;
  if (n > NUMPIXELS) n = NUMPIXELS;

  // Compute per-brake pixel brightness:
  // - if hardFlash: full red
  // - else: scale red intensity with level for a natural look
  uint8_t brakeR = hardFlash ? 255 : (uint8_t)(60 + level01 * 195); // 60..255

  // Light from center outward
  for (int k = 0; k < n; k++) {
    uint8_t idx = CENTER_OUT[k];
    strip.setPixelColor(idx, strip.Color(brakeR, 0, 0));
  }

  strip.show();
}

void setup() {
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);

  strip.begin();
  strip.setBrightness(PIXEL_BRIGHTNESS_CAP);
  strip.show();

  if (!IMU.begin()) {
    // IMU fail pattern
    while (1) {
      onboardRGB(0, 255, 0); // R+B on (active-low)
      setTailAll(); strip.show();
      delay(200);
      onboardRGB(255, 255, 255);
      for (int i=0;i<NUMPIXELS;i++) strip.setPixelColor(i, 0);
      strip.show();
      delay(200);
    }
  }

  // Idle indicator (blue) + tail
  onboardRGB(255, 255, 0);
  setTailAll();
  strip.show();
}

void loop() {
  const uint32_t now = millis();

  // -------- IMU sampling --------
  if (now - lastImu >= IMU_MS) {
    lastImu = now;

    if (IMU.accelerationAvailable()) {
      float ax, ay, az;
      IMU.readAcceleration(ax, ay, az);

      // Motion metric
      float amag = sqrtf(ax*ax + ay*ay + az*az);
      float dev  = fabsf(amag - 1.0f);
      motionMetricG = (1.0f - MOTION_ALPHA) * motionMetricG + MOTION_ALPHA * dev;

      if (!moving && motionMetricG > MOTION_ON_G)  moving = true;
      if ( moving && motionMetricG < MOTION_OFF_G) moving = false;

      // High-pass forward axis
      float aFwd = pickForwardAxis(ax, ay, az);
      aFwdLP = (1.0f - AX_LP_ALPHA) * aFwdLP + AX_LP_ALPHA * aFwd;
      float aFwdHP = aFwd - aFwdLP;

      // ----- HARD DECEL ONLY -----
      // braking intensity should respond only to deceleration along forward axis
      // Typical: braking => negative aFwdHP, so use -aFwdHP and clamp at 0
      // If your axis is reversed, set BRAKE_SIGN to -1 above.
      float decelG = (BRAKE_SIGN > 0) ? (-aFwdHP) : (aFwdHP);
      float brakeG = (decelG > 0.0f) ? decelG : 0.0f;

      // Hysteresis arm/disarm
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

      // Smooth level
      brakeLevel = (1.0f - BRAKE_SMOOTH_ALPHA) * brakeLevel + BRAKE_SMOOTH_ALPHA * target;

      // Hard flash (hard decel only)
      if (HARD_FLASH_ENABLE && brakeG > HARD_FLASH_G) {
        flashUntil = now + HARD_FLASH_MS;
      }
    }
  }

  // -------- LED update --------
  if (now - lastLed >= LED_MS) {
    lastLed = now;

    bool hardFlash = (now < flashUntil);
    bool braking = (brakeLevel > 0.12f) || hardFlash;

    setBrakeBar(brakeLevel, hardFlash);

    // Onboard RGB indicator
    if (braking) {
      onboardRGB(0, 255, 255);   // Red ON
    } else if (moving) {
      onboardRGB(255, 0, 255);   // Green ON
    } else {
      onboardRGB(255, 255, 0);   // Blue ON
    }
  }
}
