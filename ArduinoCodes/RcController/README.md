# RcController — RC receiver to dual motor PWM

This firmware runs on an **Arduino Nano** (ATmega328). It reads **two standard RC servo-style pulse channels** from a receiver (written for a **RadioLink R8EF**), mixes them **arcade / single-stick style** into left and right drive, and outputs **two independent PWM signals** for motor controllers or buffered analog drive.

**In one sentence:** it turns “steering + throttle” stick pulses into “left motor + right motor” PWM, with a defined neutral voltage and serial debug.

---

## What you need (hardware)

| Item | Role |
|------|------|
| **Arduino Nano** (5 V, ATmega328) | Runs this sketch; generates PWM on D9/D10. |
| **RC receiver** (e.g. RadioLink R8EF) | Supplies two **PPM/PWM** channels to the Nano (typically **~1000–2000 µs** pulse width, **~50 Hz** frame rate). |
| **Two servo leads or dupont wiring** | From receiver **CH for X (steering)** and **CH for Y (throttle)** to Nano **D2** and **D3**. |
| **Common ground** | Receiver GND and Nano GND must be tied together. |
| **Motor driver path** | The Nano PWM pins are **logic-level PWM**, not high-current motor power. In this project’s intended setup, **MCP602** (or similar) **buffers** scale PWM to a **~1.6 V–3.7 V** control range into your motor controller / throttle inputs—see **Output signal** below. |

No IMU, no NeoPixels—this sketch is **RC in → mixed PWM out** only.

---

## Wiring

| Nano pin | Connect to | Direction |
|----------|------------|-----------|
| **D2** | Receiver channel used for **steering (X)** | Input: PPM pulse |
| **D3** | Receiver channel used for **throttle (Y)** | Input: PPM pulse |
| **D9** | Left motor command (after your buffer/driver) | Output: PWM |
| **D10** | Right motor command (after your buffer/driver) | Output: PWM |
| **GND** | Receiver **GND** | Common reference |
| **5 V** | Power the Nano (USB or Vin per your board); receiver is often powered from the same 5 V bus or its own BEC—**do not** assume the Nano can power a heavy receiver; follow your receiver manual. |

**USB serial:** default **115200 baud** for debug prints (`X`, `Y`, `L`, `R` PWM values).

---

## Input signals (from the RC receiver)

RC receivers output **pulse-width modulation** on each channel: the **high time** of each pulse encodes stick position.

- **Nominal range:** the code expects pulses roughly in **900–2100 µs** (invalid pulses outside that window are treated as **zero** / failed read behavior in `readAxis`).
- **Calibration constants** in `src/main.cpp`: `rxMin = 1000`, `rxMid = 1500`, `rxMax = 2000` (microseconds). Sticks are mapped to a normalized **−1.0 … +1.0** per axis:
  - **Center** (`~1500 µs` ± deadband) → **0**
  - **One end** → **+1** (using `rxMax − rxMid` scaling)
  - **Other end** → **−1** (using `rxMid − rxMin` scaling)
- **Deadband:** `rxDeadband = 30` µs around center so small jitter does not move the mix.
- **Read method:** `pulseIn(..., HIGH, pulseTimeout)` with `pulseTimeout = 30000` µs—if no valid pulse in time, behavior follows `pulseIn` (effectively no update / zero-width handling as above).

So **X** is “steer left/right” and **Y** is “forward/reverse” in software; which physical stick on the transmitter maps to which receiver channel is your wiring and model setup.

---

## Mixing behavior (arcade / tank on one stick)

After reading `x` and `y` (−1…1), the sketch computes:

- `left  = y + x`
- `right = y - x`

Then it **normalizes** so that diagonal inputs do not exceed ±1:

- `maxMag = max(1, |left|, |right|)`
- `left /= maxMag`, `right /= maxMag`

**Intuition:**

- **Pure forward/back (`x ≈ 0`):** both sides get the same command (`y`).
- **Pure turn (`y ≈ 0`):** left and right oppose (`+x` vs `−x`) for **tank-style** pivoting.
- **Diagonals:** scaling keeps the larger motor demand from exceeding the normalized range.

If a channel direction feels reversed, you can change the **`invert`** argument to `readAxis` in `loop()` (see comments in `main.cpp`).

---

## Output signals (what the Nano generates)

Each output is **Arduino `analogWrite` PWM** on a timer pin (D9/D10). The sketch maps each mixed value **−1…1** to a **duty cycle** encoded as an 8-bit-style PWM value with a **neutral** in the middle:

| Meaning | PWM value (code) | Comment (for ~4.9 V rail after buffering) |
|---------|------------------|-------------------------------------------|
| Full reverse / one extreme | `minPWM` = **83** | ~**1.6 V** |
| Neutral / stop | `neutralPWM` = **140** | ~**2.7 V** |
| Full forward / other extreme | `maxPWM` = **193** | ~**3.7 V** |

Positive mixed values scale **up** from neutral toward `maxPWM`; negative values scale **down** toward `minPWM`. The **voltage table** in `main.cpp` assumes an external stage (e.g. **MCP602**) that turns this duty cycle into a **stable analog voltage** for your motor interface—**do not** connect motor loads directly to the Nano pins.

**Loop timing:** `delay(10)` → about **100 Hz** update rate (plus `pulseIn` blocking time per channel).

---

## Serial monitor

At **115200 baud**, each line looks like:

`X=… Y=… | L=… R=…`

- **X, Y:** normalized stick inputs after deadband and scaling.
- **L, R:** integer PWM values sent to D9 and D10.

Use this to verify stick directions, neutral, and mix before driving hardware.

---

## Related project

**RcAndBrakeLights** in this repo extends the same RC + PWM idea with **NeoPixel brake lights**, **MPU6050**, and compile-time **RC-only vs IMU-only** brake bar modes. This **RcController** sketch is the **minimal motor-mixing** baseline.

---

## Build / flash

PlatformIO: open this folder and build/upload for **`nanoatmega328new`** (see `platformio.ini`).
