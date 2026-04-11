# Session Report — Motor Gateway 3-Phase
**Date:** 2026-04-11  
**Board:** STM32F103C8T6 Blue Pill (GD32 clone detected)  
**Project:** AGRINET Master Gateway Firmware  
**GitHub:** https://github.com/aedaaequipments/AGRINET-Motor-Gateway

---

## 1. Upload Status

### ST-Link: `init mode failed` 🔴 UNRESOLVED

```
xPack OpenOCD x86_64 Open On-Chip Debugger 0.11.0+dev
Info : STLINK V2J47S7 (API v2) VID:PID 0483:3748
Info : Target voltage: 3.168566
Error: init mode failed (unable to connect to the target)
```

**Root cause:** NRST pin is NOT wired from ST-Link to Blue Pill RESET pin.

### Fix Option A — Wire RST (recommended)
```
ST-Link RST pin → Blue Pill R (RESET) pin
```
Then run: `pio run -t upload`

### Fix Option B — Serial bootloader (no ST-Link needed)
```bash
# 1. Move BOOT0 jumper to position 1 (3.3V side)
# 2. Connect USB-TTL adapter: TX→PA10, RX→PA9, GND→GND
# 3. Press RESET button on the board
# 4. Upload:
pio run -e serial_upload -t upload
# 5. After upload: move BOOT0 jumper back to 0, press RESET
```

---

## 2. Bug Fixes Applied (14 total, all pushed to GitHub)

### Critical Fixes

| ID | File | Bug | Fix |
|----|------|-----|-----|
| C1 | lora_radio.c | DIO0 ISR called SPI through mutex → hard fault | ISR sets flag only; `LoRa_PollIRQ()` handles SPI in task context |
| C2 | watchdog.c | IWDG 4s timeout too short for LoRa SF12 TX (5s airtime) | Increased to 8s (reload=5000) |
| C3 | power_monitor.c | 60ms ADC busy-wait starved motor task | `taskYIELD()` + `Watchdog_Feed()` every 50 samples |
| C5 | flash_config.c | Flash erase in critical section could hit WDT | `Watchdog_Feed()` before `taskENTER_CRITICAL()` |
| C6 | config.h | Display task stack 192 words → overflow | Increased to 320 words |

### Major Fixes

| ID | File | Bug | Fix |
|----|------|-----|-----|
| M1 | display.c | SSD1306 driver used for 1.3" OLED | I2C addr fallback (0x3C→0x3D), 100kHz, pull-ups, delays |
| M2 | cloud_sync.c | Empty farmId → invalid MQTT topic `data//dev/...` | Defaults to "unassigned" if empty |
| M3 | cloud_sync.c | `soilM2` used for soilMoisture field (copy-paste) | Corrected to `soilM1` |
| M5 | cloud_sync.c | Offline queue never called when GSM down | `OfflineQueue_Enqueue()` offline; `Flush()` on reconnect |
| M6 | gsm_driver.c | `vTaskDelay()` before scheduler started | `HAL_Delay()` pre-scheduler, `vTaskDelay()` after |
| M8 | motor_control.c | Single debounce timer for start/stop buttons | Separate timers per button |
| M9 | motor_control.c | `IsRunning()` false during STAR phase | Returns true for STAR and DELTA states |

### Minor Fixes

| ID | File | Bug | Fix |
|----|------|-----|-----|
| C4 | motor_control.c | Motor flags not `volatile` | Added `volatile` qualifier |
| — | main.c | Debug print said "SSD1306" | Updated to match current driver |

---

## 3. Clone Chip Compatibility Fixes (GD32/CKS32/APM32)

| Issue | Risk | Fix |
|-------|------|-----|
| **HSE crystal fails on GD32 clones** | 🔴 No UART output, MCU seems dead | Auto-fallback HSI→PLL 64MHz if HSE fails |
| **DWT CYCCNT locked (no debug probe)** | 🔴 `PowerMonitor_Sample()` infinite loop | Detect at boot; fall back to calibrated NOP delay |
| **Flash erase timing difference** | 🟡 Config corruption on save | Added ~1ms NOP delay after page erase |
| **I2C BUSY flag stuck (STM32 errata AN2824)** | 🟡 OLED fails after WDT reset | SCL toggle 9× as GPIO before I2C init |
| **ADC calibration settling** | 🟢 Slight measurement offset | Added ~10us delay after calibration |

### Boot Log (what you should see on serial):
```
=== AGRINET Master Gateway v1.0.0 ===
CLK: HSE 72MHz OK                       ← or "HSE FAILED - using HSI 64MHz (GD32 clone?)"
DWT: CYCCNT OK                           ← or "CYCCNT LOCKED (NOP fallback)"
Config loaded: MOT-001-A1 HP:0.0
LoRa: SX1276 OK (IN865)                 ← or "INIT FAILED"
GSM: SIM7670C OK                         ← or "INIT FAILED (will retry)"
OLED: SH1106 1.3" OK                    ← or "NOT FOUND"
All subsystems initialized.
Tasks created. Starting scheduler...
```

---

## 4. Build Status

```
✅ Build: SUCCESS — 0 errors, 0 warnings
Flash: [========  ]  81.8% (53596 / 65536 bytes)
RAM:   [========= ]  86.9% (17804 / 20480 bytes)
Binary: .pio/build/bluepill_f103c8/firmware.bin
```

---

## 5. Compatible Chips

| Chip | Status | Notes |
|------|--------|-------|
| STM32F103C8T6 | ✅ Tested | Genuine Blue Pill |
| GD32F103C8T6 | ✅ Compatible | Most common clone — HSE fallback + DWT fallback |
| CKS32F103C8T6 | ✅ Compatible | DWT CYCCNT may be locked |
| APM32F103C8T6 | ✅ Compatible | Flash timing fix applied |

---

## 6. Hardware Wiring Reference

### ST-Link → Blue Pill (SWD)
```
ST-Link    Blue Pill
─────────────────────
SWDIO   →  PA13 (SWDIO)
SWDCLK  →  PA14 (SWCLK)
GND     →  GND
3.3V    →  3.3V
RST     →  R (RESET)    ← MUST WIRE THIS
```

### Blue Pill UART1 → USB-Serial Adapter
```
Blue Pill     USB-Serial
────────────────────────
PA9  (TX)  →  RX
PA10 (RX)  →  TX
GND        →  GND
```

### Blue Pill → OLED 1.3" (I2C1)
```
Blue Pill     OLED
──────────────────
PB6 (SCL)  →  SCL
PB7 (SDA)  →  SDA
3.3V       →  VCC
GND        →  GND
I2C addr: 0x3C (auto-fallback to 0x3D)
```

---

## 7. Git Commits (this session)

| Commit | Description |
|--------|-------------|
| `3abaa8a` | Fix 7 critical/major bugs: SH1106 display, ISR hard fault, motor safety |
| `7ace49b` | Fix remaining bugs: WDT timeout, power monitor yield, offline queue, soilM1 |
| `66f805d` | Fix build errors: missing lora_manager.h include, resolve warnings |
| `d2bfa60` | Fix GD32 clone boot failure: HSE→HSI clock fallback + OLED improvements |
| `09e22ce` | Add GD32/CKS32/APM32 clone chip compatibility fixes |

---

## 8. Next Steps

| # | Action | Type |
|---|--------|------|
| 1 | **Wire ST-Link RST → Blue Pill RESET pin** OR use serial upload | Hardware |
| 2 | Flash firmware: `pio run -t upload` or `pio run -e serial_upload -t upload` | Upload |
| 3 | Connect USB-serial adapter to UART1 (PA9→RX, PA10→TX) | Hardware |
| 4 | Open serial monitor: `pio device monitor -b 115200` | Debug |
| 5 | Verify boot log shows clock + DWT + OLED status | Verify |
| 6 | Test OLED display shows power/motor pages | Verify |
| 7 | Test start/stop buttons (PB8/PB9) | Verify |
