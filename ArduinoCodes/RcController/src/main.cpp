#include <Arduino.h>

// RadioLink R8EF receiver -> Arduino Nano -> MCP602 buffered outputs
// Single-stick arcade/tank mixing
//
// Receiver inputs:
// D2 = X / steering channel
// D3 = Y / throttle channel
//
// Outputs:
// D9  = left motor command
// D10 = right motor command
//
// Voltage reference table for ~4.9V rail:
// 1.6 V  -> PWM  83
// 2.0 V  -> PWM 104
// 2.4 V  -> PWM 125   <-- neutral
// 2.68 V -> PWM 139
// 3.0 V  -> PWM 156
// 3.3 V  -> PWM 172
// 3.7 V  -> PWM 193

const int rxXPin = 2;
const int rxYPin = 3;

const int pwmPinLeft  = 9;
const int pwmPinRight = 10;

// Output calibration
const int minPWM     = 83;   // ~1.6V
const int neutralPWM = 125;  // ~2.4V
const int maxPWM     = 193;  // ~3.7V

const int pwmRangeUp   = maxPWM - neutralPWM;
const int pwmRangeDown = neutralPWM - minPWM;

// Receiver calibration
const int rxMin = 1000;
const int rxMid = 1500;
const int rxMax = 2000;

// Deadband around center in microseconds
const int rxDeadband = 30;

// Failsafe timeout for pulseIn, in microseconds
const unsigned long pulseTimeout = 30000UL;

float readAxis(int pin, bool invert = false) {
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
    value = (float)centered / (rxMax - rxMid);
  } else {
    value = (float)centered / (rxMid - rxMin);
  }

  value = constrain(value, -1.0f, 1.0f);

  if (invert) {
    value = -value;
  }

  return value;
}

int axisToPWM(float value) {
  value = constrain(value, -1.0f, 1.0f);

  if (value >= 0.0f) {
    return neutralPWM + (int)(value * pwmRangeUp);
  } else {
    return neutralPWM + (int)(value * pwmRangeDown);
  }
}

void setup() {
  pinMode(rxXPin, INPUT);
  pinMode(rxYPin, INPUT);

  pinMode(pwmPinLeft, OUTPUT);
  pinMode(pwmPinRight, OUTPUT);

  analogWrite(pwmPinLeft, neutralPWM);
  analogWrite(pwmPinRight, neutralPWM);

  Serial.begin(115200);
}

void loop() {
  // Adjust invert flags if stick directions come out backwards
  float x = readAxis(rxXPin, false);  // steering
  float y = readAxis(rxYPin, false);   // throttle; invert if pushing forward gives smaller pulse

  // Arcade mix
  float left  = y + x;
  float right = y - x;

  // Normalize so diagonal inputs do not exceed range
  float maxMag = max(1.0f, max(abs(left), abs(right)));
  left  /= maxMag;
  right /= maxMag;

  int pwmLeft  = axisToPWM(left);
  int pwmRight = axisToPWM(right);

  analogWrite(pwmPinLeft, pwmLeft);
  analogWrite(pwmPinRight, pwmRight);

  Serial.print("X=");
  Serial.print(x, 3);
  Serial.print(" Y=");
  Serial.print(y, 3);
  Serial.print(" | L=");
  Serial.print(pwmLeft);
  Serial.print(" R=");
  Serial.println(pwmRight);

  delay(10);
}
