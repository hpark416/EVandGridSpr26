# RcAndBrakeLights

Arduino Nano firmware that combines **RadioLink R8EF RC inputs**, **MPU6050 IMU braking**, and **NeoPixel tail/brake lighting** with an EV-style look. Motors are driven as **arcade mix** on two PWM outputs.

---

## Hardware

| Function | Pin / bus | Notes |
|----------|-----------|--------|
| RC channel X (steering) | D2 | `pulseIn` |
| RC channel Y (throttle) | D3 | `pulseIn` |
| Left motor PWM | D9 | MCP602-buffered to vehicle (per project wiring) |
| Right motor PWM | D10 | Same |
| NeoPixel data | D6 | `NUMPIXELS` = 8, `NEO_GRB` |
| MPU6050 I2C | A4 (SDA), A5 (SCL) | Address `0x68` typical |
| Status LED | D13 | Built-in; rough motion/brake hint |

Serial monitor: **115200 baud** (see `platformio.ini` → `monitor_speed`).

### Electrical interface

- **Nano supply**: Intended for a **5 V** Nano-class board; logic on D2/D3/D6/D9/D10 is **0–5 V** CMOS.
- **RC receiver (R8EF)**: Connect **channel outputs** to D2 (X) and D3 (Y). Signals are standard **RC servo PWM** (nominal **~1000–2000 µs** pulse width, ~50 Hz frame); the sketch maps that range in software (`rxMin` / `rxMid` / `rxMax`). Share **GND** with the Nano; power the receiver per its manual (often **5 V** from the same supply as the Nano if current allows).
- **Motor command (D9 / D10)**: `analogWrite` drives **~490 Hz** PWM on those pins. Firmware maps stick mix to **8-bit duty** (`minPWM` / `neutralPWM` / `maxPWM`), with comments in code tying those to roughly **~1.6 V / ~2.7 V / ~3.7 V** at the Nano pin — **your vehicle side uses an MCP602 (and any following stages)** to buffer/scale that to the real motor driver input. Do not assume the Nano pin alone can drive the motor power stage.
- **NeoPixels (D6)**: **5 V** WS2812-style strip, **data on D6**, **common ground** with the Nano. Add a **large cap** (e.g. 100 µF–1000 µF) across strip **5 V/GND** near the first pixel if your harness is long or noisy; a **series resistor** (e.g. 300–500 Ω) on the data line is often recommended.
- **MPU6050 (I2C)**: **SDA → A4**, **SCL → A5**, **GND** common. Many breakout boards include a regulator and are **5 V–friendly**; if yours is **3.3 V only**, power and I2C levels must match the module’s spec (level shifting if required).

---

## Build and upload (PlatformIO)

From this folder (`ArduinoCodes/RcAndBrakeLights`):

- Default environment: `nanoatmega328new` (new bootloader Nano).
- Alternate: `nanoatmega328` (old bootloader) — same libs and baud.

```bash
pio run -e nanoatmega328new -t upload
pio device monitor -b 115200
```

---

## High-level behavior

1. **IMU path** estimates **physical deceleration** (longitudinal jerk toward braking) and fills `brakeLevelImu` ∈ [0, 1] when `BRAKE_LIGHT_MODE` is **IMU only**.
2. **RC path** estimates **driver intent**: stick pulled for brake/reverse, and **lift-off rate** (regen-like cue). That fills `brakeLevelRc` ∈ [0, 1], smoothed with separate attack/release rates. Used for the bar when mode is **RC only** (default).
3. **Brake bar source** is **either** RC-only **or** IMU-only — set **`BRAKE_LIGHT_MODE`** in `src/main.cpp`, recompile, reflash. There is **no** fusion of the two for the bar.

4. **NeoPixels** show **cruise / reverse / brake** modes with a fixed **priority order** (see below). Forward cruise uses a **continuous phase** animation so speed changes do not “restart” the effect.

5. **RC fail-safe**: if either channel fails pulse timing (out of range), motors go to neutral PWM. In **RC only**, `brakeLevelRc` is forced to full; in **IMU only**, RC contribution is zeroed. Serial logs once per loss event.

---

## Brake modes (`BRAKE_LIGHT_MODE` in `src/main.cpp`)

You choose **one** brake-bar source at compile time — **RC-only** or **IMU-only**. They are not mixed for the bar. RC inputs (steering, throttle, reverse) still drive **motors** and **lighting layout** (forward/reverse/cruise/turn) in both modes; only **`brakeLevelRc` vs `brakeLevelImu`** changes which signal fills the brake bar and regen-style tint.

| Mode | Meaning |
|------|--------|
| `BRAKE_LIGHT_RC_ONLY` | **Default (current tuning).** Brake bar uses **only** `brakeLevelRc` (stick + lift-off). IMU still runs for motion / optional D13 hints; **NeoPixels do not use IMU hard-flash** (strip stays TX-driven). **Turn overlay:** outer pixels on the steered side **alternate** full amber vs the underlying frame (snapshot, not additive) so yellow stays visible on white reverse. Tune `TURN_*` in `main.cpp`. |
| `BRAKE_LIGHT_IMU_ONLY` | Brake bar uses **only** `brakeLevelImu` (physics braking). **Visuals mirror RC mode** where possible: cruise/regen tint, reverse white from RC stick, neutral solid red / tail, turn amber overlay from steering, IMU hard-flash on strip. **Forward cruise** also uses an **IMU motion proxy** (`imuCruiseSpeed01`) so the teal animation keeps pace when the cart is moving even with the stick near neutral (Segway-style coast). **Rapid decel blink** can trigger from a sharp IMU spike (`IMU_RAPID_DECEL_BLINK_G`) or from RC lift-off. Tune `IMU_VIS_*` / `IMU_CRUISE_SPEED_ALPHA` for your Meka mount. **Use this mode when you want to refine IMU-only behavior** while keeping the same overall light choreography as RC-only. |

### What the lights use (`displayBarLevel`)

**`displayBarLevel()`** returns **`brakeLevelRc`** (RC only) or **`brakeLevelImu`** (IMU only) with **no** neutral gating on the IMU value — idle tail vs brake uses the same thresholds as RC mode.

### Rapid forward stick drop → blinking red bar

If **forward command** drops faster than **`RC_RAPID_DROP_PER_S`** (normalized “per second”), the strip shows a **blinking red bar** for **`RC_LIFTOFF_BLINK_MS`**. This threshold is set **well above** mild/moderate lift-off (`RC_LIFTOFF_START_PER_S` / `RC_LIFTOFF_FULL_PER_S`) so only **sharp** decelerations of the stick blink. Increasing throttle does not trigger this. Tune `RC_RAPID_DROP_PER_S` in `main.cpp`.

In **IMU-only** mode, a comparable blink can also be triggered by a **sharp longitudinal decel** spike (`IMU_RAPID_DECEL_BLINK_G`); timer shares the same duration constant.

---

## IMU accuracy and hand tuning

The MPU6050 only sees **acceleration** in the sensor frame; it does not know “road braking” vs bumps, pitch, or wrong axis choice. Use **RC only** for predictable TX-matched lights, or **IMU only** if you want physics-based braking without mixing signals.

For **bench testing** (moving the board in your hand so braking is obvious), set in `main.cpp`:

- `IMU_HAND_TUNE = true`

That enables: higher decel gain (`IMU_HAND_GAIN`), lower start/full/hard-flash thresholds, and **bypasses the “must be moving” gate** so you do not have to shake the board hard enough to satisfy `moving` before IMU braking appears.

**On the vehicle**, set `IMU_HAND_TUNE = false` and adjust `FORWARD_AXIS` / `FORWARD_AXIS_SIGN` if using **IMU only**.

---

## IMU brake algorithm (detail)

Accelerometer samples are read every `IMU_MS` (5 ms). Components are normalized to **g** using `GRAVITY_MS2`.

### “Moving” detector

Uses deviation of total acceleration magnitude from 1 g:

- `motionMetricG` low-pass filters `|‖a‖ − 1|`.
- `moving` goes true/false with `MOTION_ON_G` / `MOTION_OFF_G` hysteresis.

### Longitudinal deceleration (not symmetric accel)

The code uses one axis as **forward** (`FORWARD_AXIS`: 0=X, 1=Y, 2=Z) and optional sign (`FORWARD_AXIS_SIGN` ±1).

- `aFwd` = selected axis × sign.
- `aFwdLP` = exponential low-pass of `aFwd` (`AX_LP_ALPHA`).
- `aFwdHP = aFwd − aFwdLP` (high-pass; responds to **changes** in longitudinal acceleration).

**Deceleration-only magnitude:**

`brakeG = max(0, −aFwdHP)`

So **acceleration spikes** (hitting throttle, bumps that look like forward jerk) do **not** count as braking. Earlier versions used `|aFwdHP|`, which also lit the bar on accel; that was intentional to change here.

`brakeG` is then smoothed (`BRAKE_DECEL_SMOOTH_ALPHA` → `brakeDecelGSmooth`) before thresholds.

If `IMU_BRAKE_REQUIRE_MOVING` is true, the value used for arming is **zero** when `moving` is false, reducing idle/vibration false triggers — unless `IMU_HAND_TUNE` is true (hand tuning bypasses that gate).

### Arming and bar mapping

- When `brakeUse` crosses `BRAKE_START_G`, `brakeArmed` latches true; it clears below `BRAKE_START_G − BRAKE_OFF_HYST_G`.
- When armed, a normalized target maps `brakeUse` from `[BRAKE_START_G, BRAKE_FULL_G]` to [0, 1].
- `brakeLevelImu` smooths toward that target (`BRAKE_SMOOTH_ALPHA`).

### Hard flash

If `brakeUse > HARD_FLASH_G`, `flashUntil` is set for `HARD_FLASH_MS`. The strip shows **full red IMU flash** only in **IMU-only** mode; in **RC-only** mode the NeoPixels ignore this (D13 still reflects IMU flash via `updateStatusLed`).

---

## RC brake algorithm (detail)

Channels are read every `RC_MS` (10 ms) via `pulseIn` with `pulseTimeout`. Sticks are mapped to approximately **−1…1** using `rxMin` / `rxMid` / `rxMax`. **`rxDeadband` is 0** (no stick deadband in software).

### Throttle direction (`RC_BRAKE_ON_NEGATIVE_THROTTLE`)

Defines which stick direction is “forward throttle” vs “brake/reverse” for:

- `rcForwardThrottle(y)` → [0, 1] forward command.
- `rcBrakeTargetFromThrottle(y)` → brake from **pull** (stick toward brake/reverse), with curve `RC_BRAKE_CURVE_EXP`.

### Lift-off regen cue

Each RC tick compares **current** `rcForwardThrottle(y)` to **`prevForwardThrottle`**. The drop rate (per second) maps to a brake target via `RC_LIFTOFF_START_PER_S` / `RC_LIFTOFF_FULL_PER_S`. If the drop rate exceeds **`RC_RAPID_DROP_PER_S`**, a **blinking red bar** is scheduled for **`RC_LIFTOFF_BLINK_MS`** (see NeoPixel priority).

### RC smoothing

`smoothRcBrakeToward(target)` moves `brakeLevelRc` toward `max(brakeFromStick, brakeFromLift)` with:

- `RC_BRAKE_ATTACK_ALPHA` when increasing brake,
- `RC_BRAKE_RELEASE_ALPHA` when decreasing.

### Motors

Arcade mix: `left = y + x`, `right = y − x`, normalize by max magnitude, then `axisToPWM` with asymmetric ranges above/below neutral (`minPWM` / `neutralPWM` / `maxPWM`).

---

## NeoPixel display logic (priority)

Each `LED_MS` (20 ms), the strip chooses **one** of these (first match wins):

1. **Hard IMU flash** — full red bar (`HARD_FLASH`); **IMU-only mode only** (disabled on strip in RC-only mode).
2. **Reverse display state** — white, softly pulsing (`setReverseWhite`), strength from reverse command.
3. **Rapid lift-off blink** — if a fast forward-command drop triggered `rcLiftOffBlinkUntil`, blinking red bar (`setBrakeBarBlink`); not in reverse.
4. **Forward display state** — **EV cruise animation** (`setEvCruise`): cyan/teal moving pattern; **`displayBarLevel()`** tints toward red (“regen”) using `EV_REGEN_*` so braking while still in “forward” mode keeps motion style instead of jumping to a static bar.
5. **Brake active (not forward)** — red bar grows from center (`setBrakeBar`), threshold depends on neutral vs forward (`BRAKE_DISPLAY_LEVEL_NEUTRAL` vs `BRAKE_DISPLAY_LEVEL_FORWARD`).
6. **Idle / near neutral** — dim red tail fill (`TAIL_R_BASE`).

### Display vs physical stick (hysteresis)

`rcDriveLevel` is smoothed from raw signed drive. **Display** forward/reverse uses `displayDriveState` with **enter/exit** thresholds (`DISPLAY_DRIVE_ENTER` / `DISPLAY_DRIVE_EXIT`) so LEDs do not flicker at center; this does **not** add motor deadband.

### Forward animation continuity

`setEvCruise` keeps **persistent** phase state (`evCruiseHeadPos`, `evCruiseBreathPhase`) and advances it by `LED_MS`, so when “speed” changes, the pattern **does not reset**—only the rate of phase advance changes. Forward command is also smoothed (`EV_SPEED_SMOOTH_ALPHA`). Regen red blend is smoothed separately (`EV_REGEN_SMOOTH_ALPHA`).

### Built-in LED (D13)

`updateStatusLed`: hard flash on; else on if `barLevel` > 0.05; else slow blink if `moving`; else off.

---

## Serial telemetry

At 115200, each RC tick prints something like:

`X=... Y=... | L=... R=... | Bimu=... Brc=... Bbar=...`

- `Bimu` / `Brc` — smoothed IMU and RC brake levels.
- `Bbar` — **`displayBarLevel()`** (what the strip uses).

On RC loss: `RC signal lost: neutral motor, full brake bar.`

---

## MPU6050 setup (in `setup()`)

- Range: ±4 g.
- Gyro placed in standby (accel-only use).
- Bandwidth: 21 Hz (library constant).

If `mpu.begin()` fails, the sketch **blocks** with D13 blink and a Serial message.

---

## Tuning cheat sheet (main constants)

| Constant | Role |
|----------|------|
| `FORWARD_AXIS` / `FORWARD_AXIS_SIGN` | Which accel axis is “forward” and optional flip if IMU braking feels inverted. |
| `IMU_BRAKE_REQUIRE_MOVING` | Gate IMU braking on `moving`. |
| `BRAKE_START_G`, `BRAKE_FULL_G`, `BRAKE_OFF_HYST_G` | IMU bar thresholds and hysteresis (in g-equivalent of decel signal). |
| `BRAKE_DECEL_SMOOTH_ALPHA` | Smooth raw decel before thresholds. |
| `RC_BRAKE_ON_NEGATIVE_THROTTLE` | Match your receiver’s throttle sign. |
| `DISPLAY_DRIVE_ENTER` / `EXIT` | When forward/reverse **display** modes engage (normalized stick). |
| `BRAKE_DISPLAY_LEVEL_*` | When red bar overrides cruise in neutral vs forward. |
| `EV_REGEN_RED_START` / `EV_REGEN_RED_CURVE` | How fast forward mode tints red vs `barLevel`. |
| `IMU_HAND_TUNE` / `IMU_HAND_GAIN` | Bench sensitivity; set `IMU_HAND_TUNE` false on vehicle. |
| `RC_RAPID_DROP_PER_S` | Min forward-command drop rate (per second) to trigger **blinking** red bar; raise for stricter “rapid only”. |
| `IMU_CRUISE_SPEED_ALPHA`, `IMU_VIS_FORWARD_BRAKE_MAX` | Scooter-style cruise boost from motion vs IMU brake cap for “forward” visuals. |
| `IMU_RAPID_DECEL_BLINK_G` | IMU-only: HP decel spike threshold to trigger rapid blink (parallel to RC lift-off blink). |

---

## File map

| Path | Purpose |
|------|--------|
| `src/main.cpp` | All logic: RC, IMU, NeoPixels, Serial. |
| `platformio.ini` | Board envs, libs, `monitor_speed`. |

---

## License / course use

Project lives under the course repo `EVandGridSpr26`; use and attribution per your team or instructor policy.
