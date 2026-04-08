# imuBrakeLightRR — IMU brake bar (hard decel only)

Firmware for **Arduino Nano 33 BLE Sense** that drives an **8-pixel NeoPixel** strip as a **tail + center-out brake bar**, using the **onboard IMU** (`Arduino_LSM9DS1`) and **no RC input**.

**“RR” variant:** braking responds only to **deceleration along the forward axis** (after high-pass). Positive “brake acceleration” is computed as **`-aFwdHP`** (with optional sign flip via **`BRAKE_SIGN`**). **Acceleration spikes** along forward do **not** increase the brake bar—unlike **`imuBrakeLightHP`**, which uses **`|aFwdHP|`**.

---

## Hardware

| Item | Notes |
|------|--------|
| **Arduino Nano 33 BLE Sense** | 3.3 V logic; onboard LSM9DS1. |
| **NeoPixel strip (8 LEDs)** | **D6**, `NEO_GRB` @ 800 kHz. Match voltage / level shifting to your strip. |
| **Wiring** | Data → **D6**; common **GND**; power per strip datasheet. |

**Onboard RGB** (**22 / 23 / 24**, active-low): red when braking, green when moving, blue when idle; hard flash forces red.

---

## Behavior (summary)

1. **IMU** sampling ~**5 ms**; **LED** update ~**20 ms**.
2. **Motion metric** from |a| vs **1 g** (smoothed) sets **moving** for status LEDs.
3. **`FORWARD_AXIS`** picks which of `ax, ay, az` is “forward.”
4. **High-pass** forward accel: `aFwdHP = aFwd - aFwdLP` (`AX_LP_ALPHA` controls baseline tracking).
5. **Decel-only brake:**  
   `decelG = (BRAKE_SIGN > 0) ? (-aFwdHP) : (aFwdHP)`  
   `brakeG = max(0, decelG)`  
   If braking lights on **hard acceleration**, set **`BRAKE_SIGN`** to **`-1`** in `src/main.cpp`.
6. Hysteresis and mapping **`BRAKE_START_G`…`BRAKE_FULL_G`** → smoothed **0…1** bar; optional **hard flash** above **`HARD_FLASH_G`**.
7. **Strip:** dim red tail; brake fill **center-out** `3,4,2,5,1,6,0,7`.

**IMU failure:** same fail pattern as HP project (blinking onboard + strip).

---

## Tuning

Key knobs: **`FORWARD_AXIS`**, **`BRAKE_SIGN`**, **`AX_LP_ALPHA`**, **`BRAKE_*_G`**, **`BRAKE_SMOOTH_ALPHA`**, **`HARD_FLASH_*`**, **`PIXEL_BRIGHTNESS_CAP`**.

---

## Build

PlatformIO **`nano33ble`**; deps: **Adafruit NeoPixel**, **Arduino_LSM9DS1**.

---

## Related

- **`imuBrakeLightHP`** — **magnitude** of high-pass forward (`|aFwdHP|`), direction-agnostic transients.
- **`RcAndBrakeLights`** — vehicle RC + external IMU + richer light modes on ATmega Nano.
