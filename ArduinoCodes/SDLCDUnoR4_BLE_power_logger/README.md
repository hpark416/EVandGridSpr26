# SDLCDUnoR4_BLE_power_logger — LCD + SD + BLE + Serial power logger

Firmware for **Arduino Uno R4 WiFi** that combines:

- **SDLCDUnoR4**-style **20×4 LCD**, **microSD CSV logging**, and **four-channel analog** power monitoring (battery + charger V/I).
- **BLE peripheral** and **Serial CSV** output compatible with an **Expo app “Data Logger”** workflow (same UUID/name pattern as the earlier ESP32 logger sketch).

Use this when you want **local display + SD archive** and **wireless live values** to a phone app.

---

## Android tester install guide

For APK install and validation steps (including troubleshooting), see:

- **[`TESTER_INSTRUCTIONS_ANDROID_APK.md`](TESTER_INSTRUCTIONS_ANDROID_APK.md)**

This project is intended to pair with that app via BLE for the device-connect and live telemetry flow.

---

## Compared to `SDLCDUnoR4`

| Feature | SDLCDUnoR4 | This project |
|---------|------------|----------------|
| LCD + kWh + SD CSV | Yes | Yes |
| Serial baud | 9600 | **115200** |
| Serial CSV stream | No | **Yes** (`Time_s,BatVolt_V,BatCur_A,BatPower_W,Wh`) |
| BLE | No | **Yes** |

---

## Analog inputs (same as SDLCDUnoR4)

| Pin | Variable | Notes |
|-----|----------|--------|
| **A0** | `BatVolt` | Divider **R1=11 kΩ, R2=1 kΩ** (defaults in code). |
| **A1** | `BatCur` | `(V_adc - 2.49) / 0.066` |
| **A2** | `CharVolt` | Same divider as battery voltage. |
| **A3** | `CharCur` | `(V_adc - 2.49) / 0.4` |

**ADC:** 14-bit, scaled with **5.0 / 16383**. Calibrate offsets and sensitivities for your actual sensors.

**Energy:** `KWh` integrates battery power; BLE/Serial expose **Wh** as `kWh * 1000`.

---

## Wiring

- **LCD:** RS=**3**, EN=**4**, D4–D7=**5–8**, 20×4.
- **SD:** **CS = D10** (SPI per Uno R4).
- **D2:** driven **LOW** in setup (match your schematic).
- **LED_BUILTIN:** BLE status — **blinks** when not connected, **solid ON** when a central is connected.

---

## BLE

- **Local name:** `ESP32H2-2` (kept for app compatibility with “Device 4” style pairing).
- **Service UUID:** `12345678-1234-1234-1234-1234567890ab`
- **Characteristic UUID:** `12345678-1234-1234-1234-1234567890af`
- **Payload:** ASCII CSV **`v100,i100,p100,wh100`** — integers = **×100** (e.g. `12.34 V` → `1234` as `v100` would be `1234` if using `voltage * 100` — see `updateBleAnalogValue` in `src/main.cpp`).

Central (app) should **read** the characteristic at ~**10 Hz** (firmware updates every **`SERIAL_INTERVAL_MS`** = **100 ms**).

If **`BLE.begin()`** fails, the sketch **hangs** with fast **LED blink**.

---

## Serial output (115200 baud)

Each line (battery channel only):

`Time_s,BatVolt_V,BatCur_A,BatPower_W,Wh`

`Wh` is integrated energy in **watt-hours** (from `KWh`).

---

## SD CSV

Same columns as **SDLCDUnoR4**; logged every **500 ms** when SD is present. Filename pattern **`LOG000.CSV`** … **`LOG999.CSV`**.

---

## Build

PlatformIO **`uno_r4_wifi`**, libraries: **LiquidCrystal**, **SD**, **ArduinoBLE** (`platformio.ini`).

---

## Related

**`SDLCDUnoR4`** — same display and SD logging **without** BLE or high-speed Serial CSV.
