# CAD files

Solid models, exported meshes, and related mechanical design files for the EV and Grid course project. Add or export **STEP**, **STL**, **DWG**, etc. here as the design evolves.

---

## How files map to course modules

| Files in this folder | Used in | Firmware / notes |
|----------------------|---------|------------------|
| **`RC_Electronics_Case_v1.0.stl`**, **`RC_Electronics_Lid_v1.0.stl`** | **Module 10 — Remote control cart** (`Modules/Mod10_New_Custom_Innovations/RemoteControlCart`) | Houses the **Arduino Nano** RC stack: receiver inputs **D2/D3**, motor PWM **D9/D10**. Behavior and wiring match **`ArduinoCodes/RcController`**. See that folder’s [README](../ArduinoCodes/RcController/README.md). |
| **`batteryPackCase.SLDPRT`** | **Module 9.3 — 3D printed attachments** (`Modules/Mod9_Provided_Custom_Innovations/Mod9.3_3D_Printed_Attachments`) | Battery enclosure; see the Mod 9.3 README for context. |
| **`LCD_bottom_panel.SLDPRT`**, **`LCD_Top_panel.SLDPRT`**, **`LCD_panel.SLDASM`** | **Module 9.3 — 3D printed attachments** | LCD surround / stack for the **Uno R4** power-logger builds (**`ArduinoCodes/SDLCDUnoR4`**, **`SDLCDUnoR4_BLE_power_logger`**). See [Mod 9.3 README](../Modules/Mod9_Provided_Custom_Innovations/Mod9.3_3D_Printed_Attachments/README.md). |

---

## RC electronics (Mod 10) — quick reference

The RC case STLs match a **single-stick arcade mix** on a **Nano**: steering + throttle from the receiver, left/right motor PWM out. Full pin list, signal ranges, and MCP602 buffer assumptions are documented in **`RcController`**, not duplicated here.

---

## LCD & battery CAD (Mod 9.3)

The **LCD** and **battery pack** parts are for attachments and logging hardware described under **Mod 9.3**, separate from the **Mod 10** RC cart electronics enclosure.
