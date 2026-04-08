# Module 9.3 — 3D printed attachments

This module holds **add-on parts** that mount to the vehicle or support subsystems: **battery packaging** and **LCD / logger front panel** mechanics. It is separate from **Module 10** (remote control cart electronics enclosure).

---

## CAD files (repository root: `CADfiles/`)

| CAD file | Purpose |
|----------|---------|
| **`batteryPackCase.SLDPRT`** | Enclosure / case for the battery pack assembly. |
| **`LCD_bottom_panel.SLDPRT`**, **`LCD_Top_panel.SLDPRT`**, **`LCD_panel.SLDASM`** | Panels and assembly for mounting a **20×4 character LCD** and related stack (power logger UI). |

Exports (STL, STEP) can live alongside or in this folder as you add them; the master models are tracked under **`CADfiles`**. See **[`CADfiles/README.md`](../../../CADfiles/README.md)** for the full file index.

---

## Firmware association (LCD / logger)

The **LCD** CAD aligns with the **Arduino Uno R4 WiFi** sketches that drive a **20×4** display and log to SD:

- **`ArduinoCodes/SDLCDUnoR4`** — LCD + SD CSV (battery/charger channels), no BLE.
- **`ArduinoCodes/SDLCDUnoR4_BLE_power_logger`** — same sensing and display, plus **BLE** and **Serial** streaming for an Expo **Data Logger**–style app.

Pin usage (LCD parallel, SD CS, analog front-end) is documented in each project’s **README** under `ArduinoCodes/`.

---

## Not in Mod 9.3

**RC receiver + Nano motor PWM** enclosure STLs (**`RC_Electronics_Case_v1.0.stl`**, **`RC_Electronics_Lid_v1.0.stl`**) belong to **Module 10 — Remote control cart**; see **`Modules/Mod10_New_Custom_Innovations/RemoteControlCart/README.md`**.
