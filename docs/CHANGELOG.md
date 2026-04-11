# AGRINET Motor Gateway 3-Phase - CHANGELOG

## v2.1.0 (2026-04-11) — Bug Fix Release

### Critical Fixes (C-level)

| ID  | File             | Issue                          | Fix Applied                              |
|-----|------------------|--------------------------------|------------------------------------------|
| C1  | lora_radio.c     | DIO0 ISR called SPI through mutex (hard fault) | ISR only sets flag; `LoRa_PollIRQ()` handles SPI in task context |
| C2  | watchdog.c       | IWDG 4s timeout too short for LoRa SF12 TX (5s airtime) | Increased to 8s (reload=5000) |
| C3  | power_monitor.c  | 60ms busy-wait blocked same-priority motor task | Added `taskYIELD()` + `Watchdog_Feed()` every 50 samples |
| C5  | flash_config.c   | Flash erase/write in critical section could hit WDT | `Watchdog_Feed()` called before entering critical section |
| C6  | config.h         | Display task stack 192 words caused overflow | Increased to 320 words |

### Major Fixes (M-level)

| ID  | File             | Issue                          | Fix Applied                              |
|-----|------------------|--------------------------------|------------------------------------------|
| M1  | display.c        | SSD1306 driver used for SH1106 1.3" OLED | Converted to page-mode with +2 column offset |
| M2  | cloud_sync.c     | Empty farmId created invalid MQTT topics | Defaults to "unassigned" if empty |
| M3  | cloud_sync.c     | `soilM2` used for soilMoisture field (copy-paste) | Corrected to `soilM1` |
| M5  | cloud_sync.c     | Offline queue never called when GSM down | Integrated `OfflineQueue_Enqueue()` on offline; `Flush()` on reconnect |
| M6  | gsm_driver.c     | `vTaskDelay()` called before scheduler started | Uses `HAL_Delay()` pre-scheduler, `vTaskDelay()` after |
| M8  | motor_control.c  | Single debounce timer for start/stop buttons | Separate timers for each button |
| M9  | motor_control.c  | `IsRunning()` false during STAR phase | Returns true for both STAR and DELTA states |

### Minor Fixes

| ID  | File             | Issue                          | Fix Applied                              |
|-----|------------------|--------------------------------|------------------------------------------|
| C4  | motor_control.c  | Motor flags not `volatile` (optimizer may cache) | Added `volatile` qualifier |
| —   | main.c           | Debug print said "SSD1306" after SH1106 fix | Updated to "SH1106 1.3\"" |

---

## v2.0.0 (2026-03-xx) — Initial STM32Cube + FreeRTOS Port

- Complete rewrite from Arduino+Firebase to STM32Cube HAL + FreeRTOS + MQTT
- 5 static-allocated FreeRTOS tasks (no dynamic heap)
- SX1276 LoRa IN865 (865 MHz, SF12, BW125)
- SIM7670C 4G LTE MQTT (replaces Firebase HTTP)
- True RMS power monitoring with real power factor (P = avg(v*i))
- UART provisioning system (SET_ID, SET_BROKER, SET_FARM, etc.)
- Star-delta motor control with HP-based auto timing
- Offline MQTT queue (4 packets circular buffer)
- Automation engine (soil moisture → valve + motor logic)
- Flash-persistent config with CRC-16 integrity

---

## Hardware Compatibility

| MCU                | Status  | Notes                                     |
|--------------------|---------|-------------------------------------------|
| STM32F103C8T6      | Tested  | Original Blue Pill                        |
| GD32F103C8T6       | Tested  | Clone chip — identical register map       |
| CS32F103C8T6       | Tested  | APM/CKS clone — same HAL compatibility   |

---

## Pin Configuration (unchanged from v1.0 Arduino prototype)

| Function          | Pin   | Peripheral |
|-------------------|-------|------------|
| V-CT Phase R      | PA0   | ADC1_CH0   |
| I-CT Phase R      | PA1   | ADC1_CH1   |
| V-CT Phase Y      | PA2   | ADC1_CH2   |
| I-CT Phase Y      | PA3   | ADC1_CH3   |
| V-CT Phase B      | PB0   | ADC1_CH8   |
| I-CT Phase B      | PB1   | ADC1_CH9   |
| LoRa SPI SCK      | PA5   | SPI1_SCK   |
| LoRa SPI MISO     | PA6   | SPI1_MISO  |
| LoRa SPI MOSI     | PA7   | SPI1_MOSI  |
| LoRa NSS          | PA4   | GPIO       |
| LoRa RST          | PB11  | GPIO       |
| LoRa DIO0         | PB12  | EXTI       |
| OLED SDA          | PB7   | I2C1_SDA   |
| OLED SCL          | PB6   | I2C1_SCL   |
| GSM UART TX       | PA2   | USART2_TX  |
| GSM UART RX       | PA3   | USART2_RX  |
| Debug UART TX     | PA9   | USART1_TX  |
| Debug UART RX     | PA10  | USART1_RX  |
| Button START      | PB8   | EXTI       |
| Button STOP       | PB9   | EXTI       |
| Relay MAIN        | PB3   | GPIO       |
| Relay STAR        | PB4   | GPIO       |
| Relay DELTA       | PB5   | GPIO       |
| Status LED        | PC13  | GPIO       |
