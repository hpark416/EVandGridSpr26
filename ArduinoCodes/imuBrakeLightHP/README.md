# imuBrakeLightHP — IMU brake bar (high-pass magnitude)

Firmware for **Arduino Nano 33 BLE Sense** that drives an **8-pixel NeoPixel** strip as a **tail + center-out brake bar**, using the **onboard IMU** (`Arduino_LSM9DS1`) and **no RC input**.

**“HP” variant:** braking intensity uses the **absolute value** of **high-pass–filtered** acceleration on the chosen **forward axis** (`brakeG = |aFwdHP|`). Any strong **transient** along that axis (decel *or* accel spike after filtering) can contribute—use **`imuBrakeLightRR`** if you want **hard deceleration only**.

---

## Hardware

| Item | Notes |
|------|--------|
| **Arduino Nano 33 BLE Sense** | 3.3 V logic; onboard LSM9DS1 via `Arduino_LSM9DS1`. |
| **NeoPixel strip (8 LEDs)** | **D6**, `NEO_GRB` @ 800 kHz. Power at **3.3 V** with adequate current; keep brightness modest (`PIXEL_BRIGHTNESS_CAP` in code). |
| **Wiring** | Data → **D6**; **GND** common; **3.3 V** to strip **only** if your strip is 3.3 V–tolerant (many WS2812 want 5 V level shifter—match your hardware). |

**Onboard RGB** (pins **22 / 23 / 24**, **active-low** PWM): status—idle blue, moving green, braking red hints, hard-flash sync with strip.

---

## Behavior (summary)

1. **IMU** reads `ax, ay, az` (~every **5 ms**).
2. **Motion** is inferred from deviation of |a| from **1 g** (smoothed); drives “moving” for onboard LED logic.
3. **Forward axis** is selected with **`FORWARD_AXIS`** (`0=X`, `1=Y`, `2=Z`) in `src/main.cpp`.
4. A **low-pass** of forward accel builds a baseline; **high-pass** is `aFwdHP = aFwd - aFwdLP`.
5. **`brakeG = fabsf(aFwdHP)`** → hysteresis (`BRAKE_START_G`, `BRAKE_OFF_HYST_G`) arms braking; level maps **BRAKE_START_G…BRAKE_FULL_G** → **0…1** bar fill, smoothed with **`BRAKE_SMOOTH_ALPHA`**.
6. **Hard flash:** if `brakeG > HARD_FLASH_G`, full red bar briefly (`HARD_FLASH_MS`).
7. **Strip layout:** dim red **tail** on all pixels; extra red fills **center-out** order `3,4,2,5,1,6,0,7`.

**IMU failure:** infinite blink—onboard magenta + strip alternates tail / off.

---

## Tuning

Edit constants at the top of `src/main.cpp`: **`FORWARD_AXIS`**, **`MOTION_*`**, **`AX_LP_ALPHA`**, **`BRAKE_*_G`**, **`HARD_FLASH_*`**, **`PIXEL_BRIGHTNESS_CAP`**.

---

## Build

PlatformIO, environment **`nano33ble`** (`platformio.ini`). Libraries: **Adafruit NeoPixel**, **Arduino_LSM9DS1**.

---

## Related

- **`imuBrakeLightRR`** — same hardware; **deceleration-only** braking (`BRAKE_SIGN`, no `fabs` on axis transient).
- **`RcAndBrakeLights`** — external MPU6050 + RC + optional **RC-only / IMU-only** brake bar on a 5 V Nano.
