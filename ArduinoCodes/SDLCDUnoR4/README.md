# SDLCDUnoR4 — LCD + SD power logger (Uno R4 WiFi)

Firmware for **Arduino Uno R4 WiFi** that reads **four analog channels** (battery bus + charger bus: voltage and current), shows live values on a **20×4 character LCD**, integrates **battery energy (kWh)**, and appends rows to a **CSV file on a microSD card**.

There is **no BLE** and **no high-speed Serial CSV** in this sketch—use **`SDLCDUnoR4_BLE_power_logger`** for BLE + Serial streaming to an app.

---

## What it measures

| Signal | ADC pin | In code | Typical meaning |
|--------|---------|---------|-----------------|
| Battery voltage | **A0** | `BatVolt` | Scaled with **R1/R2** divider (default **11 kΩ / 1 kΩ**). |
| Battery current | **A1** | `BatCur` | `(V_adc - 2.49 V) / 0.066` — expects a **bidirectional** sensor centered near **2.49 V** (e.g. Hall/ACS-style; **verify** your part). |
| Charger voltage | **A2** | `CharVolt` | Same divider formula as A0. |
| Charger current | **A3** | `CharCur` | `(V_adc - 2.49 V) / **0.4**` — different sensitivity than A1; **tune** for your sensor. |

**ADC:** `analogReadResolution(14)` → **0…16383**; sketch assumes **5.0 V** full-scale for conversion (`* 5.0 / 16383.0`). Adjust **`R1`/`R2`** and offset/sensitivity constants in `src/main.cpp` to match your hardware.

**Energy:** `KWh` integrates **`BatPower = BatVolt * BatCur`** over time (trapezoid-free Euler step each loop).

---

## Hardware

| Connection | Pin / interface |
|------------|-----------------|
| **LCD RS, EN, D4–D7** | **3, 4, 5, 6, 7, 8** (parallel 4-bit mode) |
| **LCD** | **20 columns × 4 rows** (`LiquidCrystal`) |
| **SD card (SPI)** | **CS = D10**; Uno R4 **SPI** pins per board (usually **11, 12, 13** + **10** CS) |
| **Analog front-end** | **A0–A3** as above |
| **D2** | Configured **OUTPUT LOW** in setup (use per your wiring—e.g. backlight or enable if tied there) |

**SD:** FAT-formatted card; sketch creates **`LOG000.CSV`**, **`LOG001.CSV`**, … up to **`LOG999.CSV`** (first free name).

---

## LCD layout

- **Row 0:** `BAT: xx.xxV` + log file id (`L000`) or `NO-SD`
- **Row 1:** `CUR: xx.xxA` + run time `MM:SS`
- **Row 2:** `PWR: xxx.xW` (battery power)
- **Row 3:** `kWh: x.xxxx` + activity spinner

---

## SD CSV format

Header:

`ms,run_s,dt_s,BatVolt_V,BatCur_A,BatPower_W,KWh,CharVolt_V,CharCur_A,CharPower_W`

Rows are written every **`LOG_INTERVAL_MS`** (**500 ms**). `dt_s` is the interval since the previous log row.

---

## Serial

**9600 baud** — minimal use in this sketch (no structured CSV stream like the BLE variant).

---

## Build

PlatformIO: board **`uno_r4_wifi`** (`renesas-ra`), libraries **LiquidCrystal**, **SD** (`platformio.ini`).

---

## Related

**`SDLCDUnoR4_BLE_power_logger`** — same sensing, LCD, and SD logging, plus **BLE** peripheral and **115200** Serial CSV for an Expo **Data Logger**–style client.
