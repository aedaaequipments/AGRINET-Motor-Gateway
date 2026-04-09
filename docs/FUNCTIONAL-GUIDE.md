# Motor Gateway 3-Phase - Functional Guide

## Device Overview

The Motor Gateway is the master IoT controller deployed at each farm. It controls 3-phase water pump motors, communicates with weather stations and valve controllers via LoRa, and syncs with the cloud via 4G LTE.

```
 ┌────────────────────────────────────────────────────────────────┐
 │              MOTOR GATEWAY - FIELD DEPLOYMENT                  │
 │                                                                │
 │                        CLOUD                                   │
 │                    (Firebase/MQTT)                              │
 │                         │                                      │
 │                    4G LTE (SIM7670C)                            │
 │                         │                                      │
 │  ┌──────────────────────┴──────────────────────────┐           │
 │  │         MOTOR GATEWAY (STM32F103C8)             │           │
 │  │                                                  │           │
 │  │  ┌────────┐  ┌────────┐  ┌────────┐            │           │
 │  │  │ Power  │  │ Motor  │  │ Display│            │           │
 │  │  │Monitor │  │Control │  │ OLED   │            │           │
 │  │  │3-Phase │  │StarDlta│  │SSD1306 │            │           │
 │  │  │ADC     │  │Relays  │  │I2C     │            │           │
 │  │  └────────┘  └────────┘  └────────┘            │           │
 │  │                                                  │           │
 │  │  ┌────────┐  ┌────────┐                         │           │
 │  │  │  LoRa  │  │ Cloud  │                         │           │
 │  │  │SX1276  │  │  Sync  │                         │           │
 │  │  │IN865   │  │GSM/4G  │                         │           │
 │  │  └───┬────┘  └────────┘                         │           │
 │  └──────┼──────────────────────────────────────────┘           │
 │         │                                                      │
 │    LoRa IN865 (5-15 km)                                        │
 │         │                                                      │
 │  ┌──────┴──────┐          ┌──────────────┐                    │
 │  │   Weather   │──UART──▶│    Valve     │                    │
 │  │   Station   │  /I2C   │  Controller  │                    │
 │  │  (STM32)    │          │   (STM8)     │                    │
 │  └─────────────┘          └──────────────┘                    │
 └────────────────────────────────────────────────────────────────┘
```

---

## 1. 3-Phase Power Monitoring

```
 ┌─────────────────────────────────────────────────────┐
 │          3-PHASE POWER MEASUREMENT                  │
 │                                                     │
 │  Mains Supply (415V 3-Phase)                        │
 │      │                                              │
 │      ├── Phase R ──▶ CT ──▶ ADC PA0 (voltage)       │
 │      │                  ──▶ ADC PA1 (current)       │
 │      │                                              │
 │      ├── Phase Y ──▶ CT ──▶ ADC PA2 (voltage)       │
 │      │                  ──▶ ADC PA3 (current)       │
 │      │                                              │
 │      └── Phase B ──▶ CT ──▶ ADC PB0 (voltage)       │
 │                         ──▶ ADC PB1 (current)       │
 │                                                     │
 │  Computed Values:                                   │
 │  ├── Per-phase: Voltage (V), Current (A), Power (W) │
 │  ├── Per-phase: Power Factor (angle detection)      │
 │  ├── Total Power: sum of 3 phases (kW)              │
 │  ├── Energy: accumulated kWh                        │
 │  └── Runtime: accumulated hours                     │
 │                                                     │
 │  Protection Thresholds:                             │
 │  ├── Overvoltage:  >260V (default)                  │
 │  ├── Undervoltage: <180V (default)                  │
 │  ├── Overload:     >I_rated × 1.3                   │
 │  ├── Dry-run:      <I_rated × 0.3                   │
 │  └── Auto-restart: 60s delay after fault clear      │
 └─────────────────────────────────────────────────────┘
```

---

## 2. Star-Delta Motor Control

```
 ┌─────────────────────────────────────────────────────┐
 │          STAR-DELTA STARTUP SEQUENCE                │
 │                                                     │
 │  START Command                                      │
 │      │                                              │
 │      ▼                                              │
 │  ┌──────────┐                                       │
 │  │   STAR   │  Main contactor + Star contactor ON   │
 │  │  (Low I) │  Motor draws ~1/3 rated current       │
 │  └────┬─────┘                                       │
 │       │  Star-delta delay (auto-adjusted by HP):    │
 │       │  2 HP → 5s, 5 HP → 8s, 10 HP → 12s         │
 │       ▼                                              │
 │  ┌──────────┐                                       │
 │  │TRANSITION│  Star OFF → 50ms dead time → Delta ON │
 │  │ (50ms)   │  Prevents short-circuit               │
 │  └────┬─────┘                                       │
 │       ▼                                              │
 │  ┌──────────┐                                       │
 │  │  DELTA   │  Main contactor + Delta contactor ON  │
 │  │ (Full I) │  Motor at full rated current           │
 │  └────┬─────┘                                       │
 │       │  Protection monitoring begins                │
 │       ▼                                              │
 │  ┌──────────┐                                       │
 │  │ RUNNING  │  Continuous V/I/PF monitoring          │
 │  │          │  Fault detection active                │
 │  └──────────┘                                       │
 │                                                     │
 │  STOP Command → All contactors OFF immediately      │
 │  FAULT Detected → Emergency stop + log + alert      │
 └─────────────────────────────────────────────────────┘
```

---

## 3. Auto-Calibration

```
 ┌─────────────────────────────────────────────────────┐
 │          MOTOR AUTO-CALIBRATION                     │
 │                                                     │
 │  First Run (or CAL command)                         │
 │      │                                              │
 │      ▼                                              │
 │  Start motor in star-delta                          │
 │      │                                              │
 │      ▼                                              │
 │  Wait for delta transition + 5s settle              │
 │      │                                              │
 │      ▼                                              │
 │  Measure steady-state delta current (I_avg)         │
 │  Measure average voltage (V_avg)                    │
 │  Measure power factor (PF)                          │
 │      │                                              │
 │      ▼                                              │
 │  Calculate HP:                                      │
 │  HP = (sqrt(3) x V_avg x I_avg x PF) / 746        │
 │      │                                              │
 │      ▼                                              │
 │  Auto-set protection thresholds:                    │
 │  ├── Overload  = I_avg × 1.3                        │
 │  ├── Dry-run   = I_avg × 0.3                        │
 │  └── Star-delta delay by HP bracket                 │
 │      │                                              │
 │      ▼                                              │
 │  Store in Flash config sector                       │
 │  App can override via timestamp comparison          │
 └─────────────────────────────────────────────────────┘
```

---

## 4. LoRa Communication

```
 ┌─────────────────────────────────────────────────────┐
 │          LoRa NETWORK                               │
 │                                                     │
 │  Motor Gateway (Master)                             │
 │      │                                              │
 │      ├── TX: Request weather data (periodic)        │
 │      ├── TX: Valve commands (open/close/position)   │
 │      ├── RX: Weather telemetry from stations        │
 │      └── RX: Valve status acknowledgments           │
 │                                                     │
 │  LoRa Config:                                       │
 │  ├── Band: IN865 (866 MHz)                          │
 │  ├── Spreading Factor: SF12 (max range)             │
 │  ├── Bandwidth: 125 kHz                             │
 │  ├── TX Power: 20 dBm                               │
 │  ├── Range: 5-15 km line of sight                   │
 │  └── Protocol: AGRINET custom framing + CRC-16      │
 │                                                     │
 │  Packet Structure:                                  │
 │  ┌────────┬──────┬────────┬─────────┬──────┐       │
 │  │ Header │ DevID│ MsgID  │ Payload │ CRC  │       │
 │  │ 25B    │ 8B   │ 2B     │ var     │ 2B   │       │
 │  └────────┴──────┴────────┴─────────┴──────┘       │
 └─────────────────────────────────────────────────────┘
```

---

## 5. OLED Display Pages

```
 ┌─────────────────────────────────────────────────────┐
 │          DISPLAY PAGES (cycle with button)          │
 │                                                     │
 │  Page 1: POWER                                      │
 │  ┌─────────────────────┐                            │
 │  │ V: 234/231/236 V    │                            │
 │  │ I: 4.2/4.1/4.3 A    │                            │
 │  │ PF: 0.86  kW: 2.8   │                            │
 │  │ kWh: 3472  Hrs: 1240 │                            │
 │  └─────────────────────┘                            │
 │                                                     │
 │  Page 2: MOTOR STATUS                               │
 │  ┌─────────────────────┐                            │
 │  │ Motor: RUNNING       │                            │
 │  │ Mode: Auto           │                            │
 │  │ HP: 5.0 (calibrated) │                            │
 │  │ S-D Delay: 8s        │                            │
 │  └─────────────────────┘                            │
 │                                                     │
 │  Page 3-5: Valves, Weather, Network                 │
 └─────────────────────────────────────────────────────┘
```

---

## 6. FreeRTOS Tasks

```
 ┌─────────────────────────────────────────────────────┐
 │          RTOS TASK MAP                              │
 │                                                     │
 │  Priority 3 (Highest):                              │
 │  ├── Task_PowerMonitor (1024B stack)                │
 │  │   ADC sampling, V/I/PF calculation               │
 │  │   Phase protection logic                         │
 │  │                                                  │
 │  └── Task_MotorControl (1536B stack)                │
 │      Star-delta FSM, relay sequencing               │
 │      Button handling (EXTI interrupt)               │
 │                                                     │
 │  Priority 2:                                        │
 │  └── Task_LoRaManager (1536B stack)                 │
 │      LoRa TX/RX, AGRINET protocol                   │
 │      Node registry (8 devices max)                  │
 │                                                     │
 │  Priority 1:                                        │
 │  └── Task_CloudSync (2048B stack)                   │
 │      GSM AT commands, MQTT publish                  │
 │      Firebase REST, config sync                     │
 │      Offline queue management                       │
 │                                                     │
 │  Priority 0 (Lowest):                               │
 │  └── Task_Display (768B stack)                      │
 │      OLED page rendering                            │
 │                                                     │
 │  Total: 5 tasks, 6912B stack, static allocation     │
 └─────────────────────────────────────────────────────┘
```

---

## Hardware Pin Map

```
 ┌─────────────────────────────────────────────────────┐
 │  STM32F103C8 PIN ASSIGNMENT                         │
 │                                                     │
 │  ADC (Power Monitoring):                            │
 │  ├── PA0: Phase R Voltage CT                        │
 │  ├── PA1: Phase R Current CT                        │
 │  ├── PA2: Phase Y Voltage CT                        │
 │  ├── PA3: Phase Y Current CT                        │
 │  ├── PB0: Phase B Voltage CT                        │
 │  └── PB1: Phase B Current CT                        │
 │                                                     │
 │  Relay Outputs (Star-Delta):                        │
 │  ├── PA8:  Main Contactor                           │
 │  ├── PB14: Star Contactor                           │
 │  └── PB15: Delta Contactor                          │
 │                                                     │
 │  LoRa SPI1 (SX1276):                               │
 │  ├── PA4: NSS (CS)                                  │
 │  ├── PA5: SCK                                       │
 │  ├── PA6: MISO                                      │
 │  ├── PA7: MOSI                                      │
 │  └── PB12: DIO0 (interrupt)                         │
 │                                                     │
 │  GSM USART3 (SIM7670C):                             │
 │  ├── PB10: TX                                       │
 │  └── PB11: RX                                       │
 │                                                     │
 │  OLED I2C1 (SSD1306):                               │
 │  ├── PB6: SCL                                       │
 │  └── PB7: SDA                                       │
 │                                                     │
 │  User Interface:                                    │
 │  ├── PB8: Button 1 (EXTI)                           │
 │  └── PB9: Button 2 (EXTI)                           │
 │                                                     │
 │  Debug: USART1 PA9/PA10 (115200 baud)               │
 └─────────────────────────────────────────────────────┘
```
