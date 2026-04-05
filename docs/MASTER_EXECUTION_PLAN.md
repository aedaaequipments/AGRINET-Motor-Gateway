# AGRINET 3-Phase Motor Controller (Master Gateway) - Master Execution Plan

## Context

The **3-Phase Motor Controller** is the **central coordination point** of the entire AGRINET smart farming platform. It is the single hardware-to-software bridge where ALL data flows through. Running on **STM32F103C8T6** (Blue Pill: 64KB Flash, 20KB SRAM, 72MHz), it handles three critical roles simultaneously:

### Role 1: Cloud Data Hub (SIM7670C 4G)
- **Pushes** 3-phase motor data (V, I, PF, kW, kWh) to Firebase
- **Pushes** weather station data received from LoRa nodes to Firebase
- **Pushes** valve status and alerts to Firebase
- **Pulls** motor ON/OFF commands from Firebase (remote control)
- **Pulls** valve open/close commands from Firebase and relays to valve nodes
- **Pulls** configurable thresholds, protection settings, star-delta timing from Firebase

### Role 2: Field Node Coordinator (LoRa SX1276)
- **Requests** weather data from Weather Station nodes periodically
- **Receives** sensor telemetry (temp, humidity, soil moisture L1/L2/L3, rain, wind, etc.)
- **Sends** valve operation commands to Valve Controller nodes
- **Receives** valve position ACK/status from valve nodes
- All inter-node communication via LoRa on IN865 band

### Role 3: Local Motor Controller + Monitor (Self-Calibrating)
- **Auto-calibrates** to any motor power rating (1HP to 10HP+)
- **Self-detects** motor power: measures delta current after star-delta transition, calculates approximate HP
- **Controls** star-delta motor starting (main/star/delta relays)
- **Monitors** 3-phase power (3x voltage CTs, 3x current CTs via ADC)
- **Protects** against voltage faults, current deviation, phase failure, dry run
- **Displays** real-time data on OLED SSD1306 (V, I, PF, status)
- **Accepts** local start/stop via physical buttons
- **Multiple run modes**: Normal, Safe Mode, Hard/Force Run

### Motor Self-Calibration (Any Power Rating)
The controller must work with ANY 3-phase motor without manual configuration:

1. **First Run / Auto-Detect**: After star-delta transition completes, measure steady-state delta current across all 3 phases
2. **HP Estimation**: `HP = (sqrt(3) x V_avg x I_avg x PF) / 746`
3. **Auto-set protection thresholds** based on detected rating:
   - Overload = rated_current x 1.3 (130%)
   - Dry run = rated_current x 0.3 (30% = no load)
   - Star-delta timing auto-adjusted by motor size (bigger motor = longer star)
4. **Store calibrated values** in flash as defaults
5. **App override**: If user sets specific values in app, those take priority and persist

### Bidirectional Config Sync (Local <-> App)
```
 FIRMWARE (Flash)              FIREBASE (App)
 ┌──────────────┐              ┌──────────────┐
 │ Default vals │──first boot──>│ Motor config │
 │ from auto-   │              │ (prot, star  │
 │ calibration  │              │  Delta, etc) │
 │              │<──app edit───│              │
 │ Updated vals │              │              │
 │ persist in   │──local edit──>│ Synced back  │
 │ flash        │              │ to app       │
 └──────────────┘              └──────────────┘

 Priority: App change > Local auto-calibration
 Sync: On every GSM cycle, compare timestamps
 Storage: Latest config always in flash (survives power loss)
```

### Operating Modes

| Mode | Behavior | How Activated |
|------|----------|---------------|
| **Normal** | All protections active, auto-calibrated thresholds | Default |
| **Safe Mode** | Extra conservative: lower overload threshold (110%), faster trip times | App toggle `safeMode=true` |
| **Hard/Force Run** | Bypasses dry-run and overload protection (voltage protection still active) | App toggle `forceRun=true`, requires confirmation |
| **Manual** | User controls start/stop only, no automation | App `mode="manual"` |
| **Auto** | Responds to soil moisture thresholds, automated start/stop | App `mode="auto"` |

### Device ID System
Each motor controller has a **unique device ID** (e.g., `MOT-001-A1`, `MOT-002-B2`). One farmer can have multiple motors across their farm. The ID is stored in STM32 flash and used for all Firebase paths.

---

## CRITICAL: Firebase Data Contract (from Sasyamithra App v6.2)

### Current vs New Firebase Structure

**OLD (prototype)** -- flat paths, single device:
```
/isRunning              -> boolean
/Motor-Values           -> {Phase1Voltage, Phase1Current, ...}
/weatherData            -> {soilTemperature, humidity, ...}
/message                -> {message: "alert text"}
```

**NEW (app v6.2)** -- user-scoped, multi-device:
```
/users/{uid}/motors/{motorId}/     -> motor state + config + protection
/users/{uid}/valves/{valveId}/     -> valve state + control modes
/users/{uid}/stations/{stationId}/ -> weather station metadata
/users/{uid}/sensorLog/{stationId}/ -> time-series sensor readings
/users/{uid}/alerts/{alertId}/     -> alerts
/users/{uid}/cmdLog/               -> command history
```

The firmware MUST use the new user-scoped paths to work with the v6.2 app.

### Motor Data the Firmware Must Push (PATCH)

Path: `/users/{uid}/motors/{motorId}.json`

```c
// Fields firmware writes to Firebase:
{
  "run": true/false,           // Motor running state
  "phV": {"r": 231, "y": 229, "b": 230},  // Phase voltages
  "phI": {"r": 4.2, "y": 4.1, "b": 4.3},  // Phase currents
  "pf":  {"o": 0.92, "r": 0.91, "y": 0.93, "b": 0.92},  // Power factors
  "pwr": 2.5,                 // Total power kW
  "kwh": 156.3,               // Cumulative energy
  "hrs": 42.5,                // Cumulative runtime hours
  "hp": "good",               // Health: "good"/"warning"/"critical"
  "startAt": "2026-03-22T10:30:00+05:30",
  "stopAt": "2026-03-22T14:00:00+05:30",
  "mode": "auto",             // Current mode
  "forceRun": false,          // Current force run state
  "safeMode": false,          // Current safe mode state

  // AUTO-CALIBRATED (firmware detects on first run, pushes as defaults):
  "config": {
    "ratedHP": 5.0,           // Auto-detected from delta current
    "ratedVoltage": 415,      // Measured line voltage
    "ratedCurrent": 7.5,      // Measured steady-state delta current
    "phases": 3,
    "calibratedAt": "2026-03-22T10:30:00+05:30"
  },
  // Auto-set protection (from calibration), overridable by app:
  "prot": {
    "dryRun": 30,             // Auto: rated_I x 0.3 timeout
    "overload": 9.75,         // Auto: rated_I x 1.3
    "overV": 260, "underV": 180, "restart": 60
  },
  "starDelta": 10             // Auto-adjusted by motor size
}
```

### Motor Config the Firmware Must Pull (GET)

Path: `/users/{uid}/motors/{motorId}.json`

```c
// Fields firmware reads from Firebase (app can override any value):
{
  "run": true,                // Remote start/stop command
  "starDelta": 10,            // Star-delta delay (seconds) -- CONFIGURABLE
  "mode": "auto"/"manual",
  "forceRun": false,          // Bypass dry-run + overload protection
  "safeMode": false,          // Extra-conservative thresholds (110%)
  "prot": {                   // Protection thresholds -- ALL CONFIGURABLE
    "dryRun": 30,             // Dry run cutoff (seconds)
    "overload": 15,           // Overload current (amps)
    "overV": 260,             // Overvoltage threshold (volts)
    "underV": 180,            // Undervoltage threshold (volts)
    "restart": 60             // Restart delay (seconds)
  }
}
// SYNC LOGIC:
// 1. First boot: firmware auto-calibrates, pushes defaults to Firebase
// 2. App edit: firmware detects newer timestamp, pulls and stores in flash
// 3. Local change: firmware pushes to Firebase with new timestamp
// 4. Power loss: flash retains latest config, syncs on next GSM connect
```

### Valve Data the Firmware Relays

Path: `/users/{uid}/valves/{valveId}.json`

```c
// Firmware reads valve commands:
{
  "isOpen": true/false,       // Valve open/close command
  "controlMode": "auto"/"manual"/"schedule"/"quantity",
  "auto": {
    "L1": {"min": 25, "max": 55, "enabled": true},  // Surface 0-10cm
    "L2": {"min": 35, "max": 70, "enabled": true},  // Root zone 15-20cm (PRIMARY)
    "L3": {"min": 30, "max": 60, "enabled": true}   // Deep 25-30cm
  }
}
// Firmware writes valve status:
{
  "isOpen": true/false,
  "flow": 12.5,              // L/min
  "togAt": "2026-03-22T10:30:00+05:30"
}
```

### Weather Data the Firmware Pushes

Path: `/users/{uid}/sensorLog/{stationId}.json` (POST = push new entry)

```c
{
  "ts": "2026-03-22T10:30:00+05:30",
  "airT": 32.1,       // Air temperature C
  "soilT": 26.8,      // Soil temperature C
  "humid": 68,         // Humidity %
  "soilM": 45.2,       // Soil moisture L2 (primary) %
  "soilM1": 40.1,      // L1 surface %
  "soilM2": 45.2,      // L2 root zone %
  "soilM3": 50.0,      // L3 deep %
  "light": 85.5,       // Light intensity %
  "leafW": 22.3,       // Leaf wetness %
  "leafW2": 18.0,      // Secondary leaf wetness %
  "wind": 4.2,         // Wind speed m/s
  "rain": 0,           // Rainfall mm
  "batt": 87           // Battery %
}
```

### Alert Data the Firmware Pushes

Path: `/users/{uid}/alerts/{alertId}.json`

```c
{
  "name": "Voltage Fault - Phase R",
  "type": "device",
  "sev": "critical",
  "at": "2026-03-22T10:30:00+05:30",
  "area": "Motor MOT-001-A1",
  "action": "Check Phase R supply voltage",
  "ack": false
}
```

---

## System Architecture

```
                    Firebase RTDB (asia-southeast1)
                    /users/{uid}/motors/{motorId}/
                    /users/{uid}/valves/{valveId}/
                    /users/{uid}/sensorLog/{stationId}/
                         |
                    [SIM7670C 4G]  USART3 (PB10/PB11)
                         |
                   Task_CloudSync (prio 1, 512w stack)
                     |       |
           cmd queue v       ^ telemetry queue
                     |       |
  Task_MotorControl      Task_LoRaManager
  (prio 3, 384w)         (prio 2, 384w)
       |                    |       |
  relays + ADC         LoRa TX    LoRa RX
       |               SPI1        SPI1
  3-phase motor     (PA4-PA7)   (PA4-PA7)
                        |           |
               Valve Nodes    Weather Nodes
               (VLV-xxx)      (WS-xxx)

  Task_PowerMonitor (prio 3, 256w) -- ADC sampling
  Task_Display (prio 0, 192w) -- OLED SSD1306 via I2C1
```

### FreeRTOS Tasks (5 tasks, static allocation)

| Task | Priority | Stack | Role |
|------|----------|-------|------|
| Task_PowerMonitor | 3 | 1024B | ADC sampling, V/I/PF calc, phase protection |
| Task_MotorControl | 3 | 1536B | Star-delta FSM, relay control, button handling |
| Task_LoRaManager | 2 | 1536B | LoRa TX/RX, AGRINET protocol, node registry |
| Task_CloudSync | 1 | 2048B | GSM AT driver, Firebase JSON, command polling |
| Task_Display | 0 | 768B | OLED page rendering |

**Total stack: 6912B** + kernel ~2KB + buffers ~4KB = ~13KB of 20KB SRAM used.

### Pin Allocation

| Pin | Function | Pin | Function |
|-----|----------|-----|----------|
| PA0 | Voltage CT R | PB0 | Current CT Y |
| PA1 | Voltage CT Y | PB1 | Current CT B |
| PA2 | Voltage CT B | PB6 | OLED SDA (I2C1) |
| PA3 | Current CT R | PB7 | OLED SCL (I2C1) |
| PA4 | LoRa NSS | PB8 | Start Button (EXTI) |
| PA5 | LoRa SCK | PB9 | Stop Button (EXTI) |
| PA6 | LoRa MISO | PB10 | GSM TX (USART3) |
| PA7 | LoRa MOSI | PB11 | GSM RX (USART3) |
| PA8 | Delta Relay | PB12 | LoRa DIO0 (EXTI) |
| PA9 | Debug TX | PB13 | LoRa RST |
| PA10 | Debug RX | PB14 | Star Relay |
| PC13 | Status LED | PB15 | Main Relay |

---

## Device ID System

Each motor controller needs a unique ID for multi-motor farms:

| Item | Format | Example | Storage |
|------|--------|---------|---------|
| Motor ID | `MOT-{seq}-{zone}` | `MOT-001-A1` | Flash (config sector) |
| Valve ID | `VLV-{seq}-{zone}` | `VLV-001-A1` | Received from app |
| Station ID | `WS-{seq}-{name}` | `WS-001-MAIN` | Received from LoRa |
| Firebase UID | User auth UID | `abc123def` | Flash (config sector) |

**Config stored in STM32 flash** (last 1KB page):
- Device ID string (16 bytes)
- Firebase user UID (32 bytes)
- Firebase auth token (48 bytes)
- Firebase DB URL (64 bytes)
- Star-delta delay (2 bytes)
- Protection thresholds (10 bytes)
- Calibration values (12 bytes)

---

## 15 Issues to Fix (from existing prototype)

| # | Issue | Fix |
|---|-------|-----|
| 1 | Firebase creds hardcoded | Store in flash config sector, provision via UART |
| 2 | LoRa on 433MHz (not IN865) | Switch to 866MHz, add channel hopping |
| 3 | No protocol headers | Integrate AGRINET packet framing + CRC-16 |
| 4 | PF hardcoded to 1.0 | Calculate real PF from V/I phase angle |
| 5 | No AGRINET UIDs | Assign motor UID, use device ID system |
| 6 | No node registry | Static array of 8 nodes with status tracking |
| 7 | No watchdog | Add IWDG (4s timeout) |
| 8 | GSM reset just re-inits | Proper state machine with exponential backoff |
| 9 | Only 20KB heap declared | Static allocation only, no dynamic heap |
| 10 | String-heavy JSON | Use `snprintf` char arrays, no String class |
| 11 | No OTA | Add OTA framework (Phase 3) |
| 12 | LoRa+GSM in same task | Split into Task_LoRaManager + Task_CloudSync |
| 13 | No timestamps in Firebase | Get time from GSM (AT+CCLK?) |
| 14 | No local automation | Add rule-based automation using app thresholds |
| **15** | **Wrong Firebase paths** | **Migrate from flat to user-scoped paths** |

---

## Phased Execution Roadmap

### Phase 1: v1.0 -- Foundation (Weeks 1-3)

**Files to create:**
- `src/main.cpp` -- setup(), task creation, FreeRTOS start
- `src/config.h` -- pins, constants, feature flags, device ID struct
- `src/credentials.h.template` -- Firebase URL/UID/token template
- `src/power_monitor.cpp/.h` -- ADC, EmonLib, real PF calculation
- `src/motor_control.cpp/.h` -- star-delta FSM, relay control, protection
- `src/watchdog.cpp/.h` -- IWDG init/feed
- `src/flash_config.cpp/.h` -- STM32 flash read/write for persistent config
- `platformio.ini` -- STM32F103C8, FreeRTOS, stlink

**Tasks:**
1. Create PlatformIO project (STM32F103C8, STM32Cube + FreeRTOS)
2. Port pin definitions from prototype to `config.h`
3. Implement device ID system with flash storage
4. Port EmonLib power monitoring, **fix PF calculation** (per-phase PF)
5. **Motor auto-calibration system:**
   - After star-delta transition, measure steady-state delta current (3 phases)
   - Calculate HP: `HP = (sqrt(3) x V_avg x I_avg x PF) / 746`
   - Auto-set protection: overload=I_rated*1.3, dryRun=I_rated*0.3
   - Auto-adjust star-delta timing by motor size (2HP=5s, 5HP=8s, 10HP=12s)
   - Store calibrated values in flash as defaults
6. Port star-delta FSM with **configurable delay** (auto-set or from app)
7. Implement **configurable protection thresholds** (overV, underV, overload, dryRun, restart)
8. Implement **operating modes**: Normal, Safe Mode, Hard/Force Run
   - Safe Mode: overload threshold = rated_I x 1.1 (tighter), faster trip
   - Force Run: bypasses dry-run and overload checks (voltage protection stays active)
9. **Flash config manager**: read/write persistent config sector
   - Device ID, Firebase UID, auth token, DB URL
   - Calibrated motor values, protection thresholds, star-delta delay
   - Mode flags (safeMode, forceRun)
   - Timestamp for config sync
10. Add IWDG watchdog
11. Configure FreeRTOS with **static allocation only**
12. Create 5 tasks with queues and mutexes
13. Basic OLED display (power readings + motor status + device ID + mode)
14. UART provisioning command to set device ID, Firebase UID, and auth token

**Deliverable:** Board auto-calibrates to any motor, runs with configurable protection, multiple modes, persistent config, watchdog, and device identity.

### Phase 2: v2.0 -- Communication (Weeks 4-7)

**Files to create:**
- `src/lora_manager.cpp/.h` -- SX1276 register-level driver
- `src/agrinet_protocol.h` -- copied from shared include
- `src/cloud_sync.cpp/.h` -- SIM7670C AT state machine, Firebase REST
- `src/node_registry.cpp/.h` -- device tracking
- `src/firebase_json.cpp/.h` -- compact JSON builder matching app schema

**Tasks:**
1. Write register-level SX1276 driver (not RadioLib -- too large for 64KB)
2. **Switch to IN865 band** (866MHz, 3 channels, duty cycle tracking)
3. Implement AGRINET protocol framing (25-byte header + payload + CRC-16)
4. Build node registry: 8 nodes, 32 bytes each
5. **Valve integration** via LoRa or direct I2C
6. **Weather station integration**: parse sensor data, report 3 soil moisture levels (L1/L2/L3)
7. Rewrite GSM driver as state machine with error recovery
8. **Firebase JSON builder matching app v6.2 schema:**
   - PATCH `/users/{uid}/motors/{motorId}` with phV, phI, pf, pwr, kwh, hrs, run, hp
   - GET `/users/{uid}/motors/{motorId}` for run command + protection config + starDelta
   - POST `/users/{uid}/sensorLog/{stationId}` with weather readings
   - GET `/users/{uid}/valves/{valveId}` for valve commands + auto thresholds
   - PATCH `/users/{uid}/valves/{valveId}` with isOpen, flow, togAt
   - POST `/users/{uid}/cmdLog` for command logging
   - POST `/users/{uid}/alerts` for fault alerts
9. Add timestamps from GSM network time (AT+CCLK?)
10. **Split LoRa+GSM tasks**
11. Implement **heartbeat** -- update motor `lastSeen` and `online` status

**Deliverable:** End-to-end data flow matching Sasyamithra app v6.2 schema.

### Phase 3: v3.0 -- Intelligence & Hardening (Weeks 8-12)

**Files to create:**
- `src/automation.cpp/.h` -- rule engine using app thresholds
- `src/display.cpp/.h` -- multi-page OLED
- `src/ota.cpp/.h` -- firmware update framework

**Tasks:**
1. **Local automation engine** using app-configured thresholds:
   - Read `valves/{id}/auto/L1,L2,L3` min/max from Firebase
   - Compare with weather station soil moisture readings
   - Auto-open valve if L2 (root zone) < min, close if L2 > max
   - Environmental adjustment: modify based on humidity, leaf wetness
   - Pest/disease risk calculation (humidity >85% = +30 risk, etc.)
2. Multi-page OLED display (5 pages: Power, Motor, Valves, Weather, Network)
3. Firebase offline queue (buffer 8 packets when GSM is down)
4. **Energy metering** with persistence (kWh in flash, push to Firebase `kwh` field)
5. **Motor runtime tracking** (hours in flash, push to Firebase `hrs` field)
6. **Motor health assessment** (compute `hp` field: good/warning/critical based on PF + deviation)
7. OTA framework via Firebase
8. Production hardening:
   - Stack overflow hooks
   - Brown-out detection
   - Persistent fault logging
   - GSM signal quality monitoring
   - IN865 duty cycle enforcement
9. 24-hour stability test

**Deliverable:** Field-deployable firmware with full app integration.

---

## Extra Features Needed (from App Analysis)

| Feature | App Expects | Firmware Must Do | Phase |
|---------|-------------|-----------------|-------|
| **Auto-calibration** | `config.ratedHP/ratedCurrent` | Detect motor HP from delta current, set defaults | v1.0 |
| **Bidirectional sync** | Config with timestamps | Flash <-> Firebase sync, latest wins | v1.0 |
| **Safe mode** | `safeMode` toggle | Tighter thresholds (110%), faster trip | v1.0 |
| **Force/hard run** | `forceRun` toggle | Bypass dry-run + overload, keep voltage prot | v1.0 |
| Device ID | `MOT-001-A1` format | Store in flash, send in all Firebase paths | v1.0 |
| Configurable star-delta | `starDelta` field (seconds) | Auto-set by HP or from app, apply to FSM | v1.0 |
| Protection thresholds | `prot.overV/underV/overload/dryRun/restart` | Auto-set from calibration or from app | v1.0 |
| Real Power Factor | `pf.o/r/y/b` (0-1 per phase) | Calculate from ADC, push to Firebase | v1.0 |
| Per-phase PF | `pf.r, pf.y, pf.b` | Individual phase angle detection | v1.0 |
| Energy metering | `kwh` field | Accumulate, persist in flash | v3.0 |
| Runtime hours | `hrs` field | Track, persist in flash | v3.0 |
| Motor health | `hp` = good/warning/critical | Derive from PF + current deviation | v3.0 |
| 3-level soil moisture | `soilM1, soilM2, soilM3` | Parse from weather station, push separately | v2.0 |
| Auto irrigation | `auto.L1/L2/L3` thresholds | Compare soil moisture vs thresholds | v3.0 |
| Valve control modes | `controlMode` = auto/manual/schedule/qty | Read mode, execute appropriate logic | v2.0 |
| Command logging | POST to `cmdLog` | Log every motor start/stop, valve toggle | v2.0 |
| Alert system | POST to `alerts` with type/sev | Push structured alerts on faults | v2.0 |
| Heartbeat | `connectivity.lastSeen/online` | Periodic Firebase update | v2.0 |

---

## Memory Budget

### Flash (64KB)
| Module | Est. Size |
|--------|-----------|
| FreeRTOS kernel | 7 KB |
| LoRa SX1276 (register-level) | 4 KB |
| AGRINET protocol + CRC | 2 KB |
| EmonLib (3-phase + PF) | 4 KB |
| SIM7670C AT driver | 6 KB |
| Motor FSM + protection | 3 KB |
| Firebase JSON builder (app schema) | 4 KB |
| SSD1306 OLED (page-mode) | 3 KB |
| Node registry + automation | 3 KB |
| Flash config + provisioning | 2 KB |
| Main + glue + watchdog | 3 KB |
| **Total** | **~41 KB (64%)** |

### SRAM (20KB)
| Item | Size |
|------|------|
| Task stacks (5 tasks) | 6912 B |
| FreeRTOS kernel objects | 2048 B |
| GSM RX buffer | 512 B |
| LoRa packet buffer | 256 B |
| JSON scratch buffer | 512 B |
| ADC arrays (6ch x 150 samples) | 1800 B |
| Node registry (8 nodes) | 256 B |
| OLED page buffer | 256 B |
| Motor config struct | 128 B |
| Device ID + Firebase paths | 256 B |
| Globals + state | 512 B |
| **Total** | **~13.4 KB (67%)** |

---

## Key Reference Files

| File | Purpose |
|------|---------|
| `Projects/sasyamithra-pilot/docs/DATABASE_SCHEMA.md` | Full Firebase schema |
| `Projects/sasyamithra-pilot/src/types/database.ts` | TypeScript data models |
| `Projects/sasyamithra-pilot/src/pages/MotorsPage.tsx` | Motor UI + settings |
| `Projects/sasyamithra-pilot/src/pages/ValvesPage.tsx` | Valve UI + control modes |
| `Projects/sasyamithra-pilot/src/contexts/DeviceContext.tsx` | Firebase device paths |
| `ws-cld-ai-v3/include/agrinet_protocol.h` | Shared LoRa protocol |
| `valve-v3/AGRINET-Valve-Controller/src/config.h` | Valve I2C register map |
| `valve-v3/examples/I2C_Master_Test/` | I2C master reference |
| `tpm-v1-test-final.ino` | Working prototype to port from |

---

## Risks & Mitigations

| Risk | Mitigation |
|------|-----------|
| 20KB SRAM too tight | Static alloc only, page-mode OLED, reduce ADC samples if needed |
| 64KB Flash insufficient | Register-level drivers (not RadioLib 20KB+), strip debug strings |
| Blue Pill counterfeit STM32 | Test actual flash; many have 128KB despite 64KB label |
| GSM failures block task | Separate task, per-command timeouts, exponential backoff |
| ADC noise from relay switching | Sample only when relays stable, moving-average filter |
| Star-delta overlap damages motor | Enforce 50ms dead-time, never star+delta simultaneously |
| IN865 duty cycle violation | Track TX time per channel, enforce 1% in software |
| Firebase auth token expiry | Implement token refresh or use database secret for device auth |
| App schema changes break firmware | Version the JSON builder, test against app data models |

---

## Verification Plan

| Test | Pass Criteria |
|------|---------------|
| **Auto-calibration** | Connect to 2HP motor -> detects ~2HP; connect to 5HP -> detects ~5HP |
| **Auto protection** | Calibrated overload = rated_I x 1.3, dryRun triggers at rated_I x 0.3 |
| **Config sync** | Change threshold in app -> firmware picks up within 30s, stores in flash |
| **Config persist** | Power cycle -> firmware boots with last synced config from flash |
| **Safe mode** | Toggle in app -> overload drops to rated_I x 1.1, trips faster |
| **Force run** | Toggle in app -> motor runs despite low current (dry-run bypassed) |
| **Force run safety** | Force run still trips on overvoltage (voltage prot never bypassed) |
| Power accuracy | V within 2%, I within 5% vs calibrated meter |
| Power factor | PF within 0.05 of reference meter, per-phase values correct |
| Star-delta transition | Auto-adjusted delay by HP, 50ms dead-time, no overlap |
| Protection trip | Stops within 2s of threshold breach (overV/underV/overload) |
| Device ID provisioning | Set ID via UART, persists across power cycle |
| Firebase motor data | App shows correct V/I/PF/kW matching firmware readings |
| Firebase valve command | App toggles valve, firmware relays via LoRa, valve ACKs |
| Firebase weather data | App shows sensor readings matching LoRa-received data |
| 3-level soil moisture | App shows L1/L2/L3 from sensorLog correctly |
| Auto irrigation | L2 drops below min -> valve opens, rises above max -> closes |
| Alert delivery | Voltage fault -> alert appears in app within 30s |
| Command logging | Motor start/stop logged in cmdLog |
| LoRa range | Reliable at 500m with SF12 |
| GSM recovery | Auto-reconnects within 120s |
| Memory stability | 24-hour run, no leak, stack watermarks stable |
| Watchdog recovery | Reset within 4s on hang |
| Multi-motor | Two boards with different IDs work independently on same Firebase |
