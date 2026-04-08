# Remote control cart — electronics and CAD

This folder supports **Module 10 — New Custom Innovations**: a **radio-controlled cart** driven by an **Arduino Nano** reading a **two-channel RC receiver** and outputting **dual motor PWM** (arcade / tank mix on one stick).

---

## Firmware (source of truth for behavior)

All **signal definitions, wiring, mixing math, and PWM scaling** are documented in:

**[`ArduinoCodes/RcController/README.md`](../../../ArduinoCodes/RcController/README.md)**

The sketch **`RcController`** (`ArduinoCodes/RcController/src/main.cpp`) assumes:

| Nano | Role |
|------|------|
| **D2** | Receiver **steering (X)** — PPM ~1000–2000 µs |
| **D3** | Receiver **throttle (Y)** — PPM ~1000–2000 µs |
| **D9** | **Left** motor command (PWM → buffered ~1.6–3.7 V control voltage) |
| **D10** | **Right** motor command (same) |
| **GND** | Common with receiver |

Serial debug defaults to **115200 baud** (`X`, `Y`, `L`, `R` PWM). Receiver model in comments: **RadioLink R8EF** (any compatible PPM receiver can work with calibration).

**Mixing:** `left = y + x`, `right = y - x`, normalized so diagonals do not exceed full scale—see the RcController README for detail.

---

## Mechanical / CAD

Printed enclosure parts for this stack live under the repo **`CADfiles`** folder:

| File | Role |
|------|------|
| **`CADfiles/RC_Electronics_Case_v1.0.stl`** | Main box for Nano + receiver + wiring |
| **`CADfiles/RC_Electronics_Lid_v1.0.stl`** | Lid |

See **[`CADfiles/README.md`](../../../CADfiles/README.md)** for how these files relate to Mod 9 vs Mod 10.

---

## Other contents here

- **`ReverseEngineeringPics/`** — reference photos for the cart / RC integration.
- **`joystick.xlsx`** — stick / geometry or channel notes (spreadsheet).

---

## Related projects in this repo

- **`RcAndBrakeLights`** — same RC + motor path extended with **NeoPixels**, **MPU6050**, and **RC-only vs IMU-only** brake bar modes on a vehicle build.
