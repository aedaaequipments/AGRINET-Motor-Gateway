/**
 * ═══════════════════════════════════════════════════════════════════════════════
 * SASYAMITHRA-AGRINET IoT Platform
 * LoRa Protocol Definitions
 * ═══════════════════════════════════════════════════════════════════════════════
 * 
 * This header defines the LoRa communication protocol used across all AGRINET
 * nodes. Include this file in all firmware projects.
 * 
 * @file    agrinet_protocol.h
 * @version 3.1.0
 * @date    2026-03-20
 * @author  AEDAA - UR Farm Assistant
 * 
 * ═══════════════════════════════════════════════════════════════════════════════
 */

#ifndef AGRINET_PROTOCOL_H
#define AGRINET_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// ═══════════════════════════════════════════════════════════════════════════════
// PROTOCOL VERSION
// ═══════════════════════════════════════════════════════════════════════════════

#define AGRINET_PROTOCOL_VERSION    0x01

// ═══════════════════════════════════════════════════════════════════════════════
// IN865 LORA CONFIGURATION (INDIA)
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * WPC (Wireless Planning Commission) IN865 Band Specifications
 * - Frequency Range: 865-867 MHz
 * - Max TX Power: 14 dBm conducted (30 dBm EIRP with 6dBi antenna)
 * - Duty Cycle: 1% per channel (36 seconds per hour)
 */

#define LORA_FREQUENCY_DEFAULT      866000000   // 866.0 MHz center
#define LORA_BANDWIDTH              125000      // 125 kHz
#define LORA_SPREADING_FACTOR       12          // SF12 for max range
#define LORA_CODING_RATE            5           // 4/5
#define LORA_TX_POWER               20          // 20 dBm with PA
#define LORA_SYNC_WORD              0x34        // Private network
#define LORA_PREAMBLE_LENGTH        8

// IN865 Mandatory Channels (WPC compliant hopping)
#define IN865_CHANNEL_0             865062500   // 865.0625 MHz
#define IN865_CHANNEL_1             865402500   // 865.4025 MHz
#define IN865_CHANNEL_2             865985000   // 865.985 MHz
#define IN865_CHANNEL_COUNT         3

#define DUTY_CYCLE_PERCENT          1           // 1% per channel

// ═══════════════════════════════════════════════════════════════════════════════
// DEVICE UID FORMAT
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * 64-bit Device Unique Identifier Format:
 * 
 * ┌────────────────┬──────────┬────────────────────────────────┐
 * │   OUI (24-bit) │Type (8b) │       Serial (32-bit)          │
 * ├────────────────┼──────────┼────────────────────────────────┤
 * │  0x41 47 52    │   'M'    │       0x00000001               │
 * │    "AGR"       │          │                                │
 * └────────────────┴──────────┴────────────────────────────────┘
 * 
 * Human-readable format: AGN-M-00000001
 */

#define AGRINET_OUI_BYTE0           0x41    // 'A'
#define AGRINET_OUI_BYTE1           0x47    // 'G'
#define AGRINET_OUI_BYTE2           0x52    // 'R'

// Device type codes
#define NODE_TYPE_MASTER            'M'     // Master Gateway
#define NODE_TYPE_WEATHER           'W'     // Weather Station
#define NODE_TYPE_SOIL              'S'     // Soil Sensor
#define NODE_TYPE_VALVE             'V'     // Valve Controller
#define NODE_TYPE_MOTOR             'P'     // Motor/Pump Controller
#define NODE_TYPE_RELAY             'R'     // Generic Relay Node

// Broadcast UID
#define UID_BROADCAST               0xFFFFFFFFFFFFFFFFULL

// ═══════════════════════════════════════════════════════════════════════════════
// PACKET STRUCTURE
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * LoRa Packet Structure:
 * 
 * ┌──────────────────────────────────────────────────────────────┐
 * │                     LoRa Header (25 bytes)                   │
 * ├──────────┬────────┬───────┬─────────┬─────────┬─────────────┤
 * │ Sync (2) │ Len(1) │Flg(2) │ Src(8)  │ Dst(8)  │  MsgID(4)   │
 * ├──────────┴────────┴───────┴─────────┴─────────┴─────────────┤
 * │                    Payload (0-230 bytes)                     │
 * ├──────────────────────────────────────────────────────────────┤
 * │                     CRC-16 (2 bytes)                         │
 * └──────────────────────────────────────────────────────────────┘
 */

#define LORA_SYNC_WORD_PACKET       0xAA55
#define LORA_MAX_PACKET_SIZE        255
#define LORA_HEADER_SIZE            25
#define LORA_CRC_SIZE               2
#define LORA_MAX_PAYLOAD_SIZE       (LORA_MAX_PACKET_SIZE - LORA_HEADER_SIZE - LORA_CRC_SIZE)

// LoRa Header Structure
typedef struct {
    uint16_t sync;          // Sync word: 0xAA55
    uint8_t  length;        // Total packet length (header + payload + CRC)
    uint16_t flags;         // Header flags (see below)
    uint64_t srcUid;        // Source device UID
    uint64_t dstUid;        // Destination UID (0xFFFF... = broadcast)
    uint32_t msgId;         // Unique message ID (incrementing counter)
} __attribute__((packed)) LoRaHeader_t;

// ═══════════════════════════════════════════════════════════════════════════════
// HEADER FLAGS
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * Header Flags (16-bit):
 * 
 * ┌───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┐
 * │15 │14 │13 │12 │11 │10 │ 9 │ 8 │ 7 │ 6 │ 5 │ 4 │ 3 │ 2 │ 1 │ 0 │
 * ├───┼───┼───┼───┴───┴───┴───┼───┴───┴───┴───┼───┴───┴───┼───┴───┤
 * │ENC│FRG│ACK│   Max Hops    │   Hop Count   │Frame Type │ Ver   │
 * └───┴───┴───┴───────────────┴───────────────┴───────────┴───────┘
 */

// Version (bits 0-1)
#define FLAG_VERSION_MASK           0x0003
#define FLAG_VERSION_1              0x0001

// Frame type (bits 2-4)
#define FLAG_TYPE_MASK              0x001C
#define FLAG_TYPE_SHIFT             2
#define FRAME_TYPE_DATA             0       // Sensor telemetry
#define FRAME_TYPE_CMD              1       // Command to actuator
#define FRAME_TYPE_ACK              2       // Acknowledgment
#define FRAME_TYPE_NACK             3       // Negative acknowledgment
#define FRAME_TYPE_HELLO            4       // Discovery beacon
#define FRAME_TYPE_CONFIG           5       // Configuration update
#define FRAME_TYPE_OTA              6       // Firmware chunk
#define FRAME_TYPE_ALERT            7       // Urgent alert

// Hop count (bits 5-8)
#define FLAG_HOP_COUNT_MASK         0x01E0
#define FLAG_HOP_COUNT_SHIFT        5

// Max hops (bits 9-12)
#define FLAG_MAX_HOPS_MASK          0x1E00
#define FLAG_MAX_HOPS_SHIFT         9
#define DEFAULT_MAX_HOPS            3

// Control flags (bits 13-15)
#define FLAG_ACK_REQUIRED           0x2000  // Bit 13: ACK required
#define FLAG_FRAGMENTED             0x4000  // Bit 14: Fragmented packet
#define FLAG_ENCRYPTED              0x8000  // Bit 15: Encrypted payload

// ═══════════════════════════════════════════════════════════════════════════════
// PAYLOAD STRUCTURES
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * Weather Station Payload (32 bytes)
 * Sent by AGN-W nodes every wake cycle
 */
typedef struct {
    int16_t  airTemp;       // Air temperature x100 (°C)
    uint16_t humidity;      // Relative humidity x100 (%)
    uint16_t pressure;      // Barometric pressure - 900 (hPa offset)
    uint16_t lightLux;      // Light intensity / 10 (0-65535 lux)
    uint16_t soilM1;        // Soil moisture L1 x100 (VWC%)
    uint16_t soilM2;        // Soil moisture L2 x100 (VWC%)
    uint16_t soilM3;        // Soil moisture L3 x100 (VWC%)
    int16_t  soilTemp;      // Soil temperature x100 (°C)
    uint8_t  leafWet1;      // Leaf wetness 1 (0-100%)
    uint8_t  leafWet2;      // Leaf wetness 2 (0-100%)
    uint16_t rainMm;        // Rainfall x10 (mm)
    uint16_t windSpeed;     // Wind speed x100 (m/s)
    uint16_t windDir;       // Wind direction (0-359°)
    uint8_t  uvIndex;       // UV index x10
    uint16_t batteryMv;     // Battery voltage (mV)
    int8_t   rssi;          // Last received RSSI (dBm)
    uint8_t  faults;        // Sensor fault flags
} __attribute__((packed)) WeatherPayload_t;

/**
 * Valve Controller Payload (12 bytes)
 * Sent by AGN-V nodes on state change or poll
 */
typedef struct {
    uint8_t  valveId;       // Valve index (0-15)
    uint8_t  position;      // Current position (0=closed, 100=open)
    uint8_t  targetPos;     // Target position
    uint8_t  state;         // State machine state
    uint16_t flowRate;      // Flow rate x10 (L/min)
    uint32_t totalVolume;   // Total volume (liters)
    uint16_t batteryMv;     // Battery voltage (mV)
} __attribute__((packed)) ValvePayload_t;

/**
 * Motor Controller Payload (20 bytes)
 * Sent by AGN-P nodes periodically
 */
typedef struct {
    uint16_t voltageR;      // Phase R voltage x10 (V)
    uint16_t voltageY;      // Phase Y voltage x10 (V)
    uint16_t voltageB;      // Phase B voltage x10 (V)
    uint16_t currentR;      // Phase R current x100 (A)
    uint16_t currentY;      // Phase Y current x100 (A)
    uint16_t currentB;      // Phase B current x100 (A)
    uint16_t activePower;   // Active power x100 (kW)
    uint16_t powerFactor;   // Power factor x1000
    uint32_t energyKwh;     // Energy x100 (kWh)
    uint8_t  motorState;    // 0=OFF, 1=STAR, 2=DELTA, 3=FAULT
    uint8_t  faults;        // Fault flags
} __attribute__((packed)) MotorPayload_t;

/**
 * Command Payload (16 bytes max)
 * Sent by Master to actuator nodes
 */
typedef struct {
    uint8_t  cmdType;       // Command type (see CMD_TYPE_*)
    uint8_t  target;        // Target index (valve#, relay#, etc.)
    uint16_t value;         // Command value
    uint32_t duration;      // Duration in seconds (0 = indefinite)
    uint32_t timestamp;     // Command timestamp (Unix)
    uint32_t reserved;      // Reserved for future use
} __attribute__((packed)) CommandPayload_t;

// Command types
#define CMD_TYPE_VALVE_SET          0x01    // Set valve position
#define CMD_TYPE_MOTOR_START        0x02    // Start motor
#define CMD_TYPE_MOTOR_STOP         0x03    // Stop motor
#define CMD_TYPE_RELAY_SET          0x04    // Set relay state
#define CMD_TYPE_CONFIG_UPDATE      0x10    // Update configuration
#define CMD_TYPE_REBOOT             0xFE    // Reboot device
#define CMD_TYPE_OTA_START          0xFF    // Start OTA update

// ═══════════════════════════════════════════════════════════════════════════════
// FAULT FLAGS
// ═══════════════════════════════════════════════════════════════════════════════

// Weather Station Faults
#define FAULT_TEMP_HUMID            0x01    // AHT20 failure
#define FAULT_PRESSURE              0x02    // BMP280 failure
#define FAULT_LIGHT                 0x04    // BH1750 failure
#define FAULT_SOIL_MOISTURE         0x08    // Soil sensor failure
#define FAULT_SOIL_TEMP             0x10    // DS18B20 failure
#define FAULT_LEAF_WETNESS          0x20    // Leaf sensor failure
#define FAULT_RAIN                  0x40    // Rain gauge failure
#define FAULT_BATTERY_LOW           0x80    // Battery < 3.3V

// Motor Controller Faults
#define MOTOR_FAULT_OVERLOAD        0x01    // Current > threshold
#define MOTOR_FAULT_UNDERVOLT       0x02    // Voltage < 180V
#define MOTOR_FAULT_OVERVOLT        0x04    // Voltage > 260V
#define MOTOR_FAULT_PHASE_FAIL      0x08    // Phase missing
#define MOTOR_FAULT_DRY_RUN         0x10    // No load detected
#define MOTOR_FAULT_THERMAL         0x20    // Thermal protection
#define MOTOR_FAULT_CONTACTOR       0x40    // Contactor feedback fail
#define MOTOR_FAULT_COMM            0x80    // Communication timeout

// ═══════════════════════════════════════════════════════════════════════════════
// CRC CALCULATION
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * Calculate CRC-16-CCITT checksum
 * Polynomial: 0x1021
 * Initial: 0xFFFF
 * 
 * @param data Pointer to data buffer
 * @param length Length of data
 * @return CRC-16 checksum
 */
static inline uint16_t agrinet_crc16(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

// ═══════════════════════════════════════════════════════════════════════════════
// HELPER MACROS
// ═══════════════════════════════════════════════════════════════════════════════

// Build UID from components
#define AGRINET_MAKE_UID(type, serial) \
    (((uint64_t)AGRINET_OUI_BYTE0 << 56) | \
     ((uint64_t)AGRINET_OUI_BYTE1 << 48) | \
     ((uint64_t)AGRINET_OUI_BYTE2 << 40) | \
     ((uint64_t)(type) << 32) | \
     ((uint64_t)(serial) & 0xFFFFFFFF))

// Extract frame type from flags
#define AGRINET_GET_FRAME_TYPE(flags) \
    (((flags) & FLAG_TYPE_MASK) >> FLAG_TYPE_SHIFT)

// Extract hop count from flags
#define AGRINET_GET_HOP_COUNT(flags) \
    (((flags) & FLAG_HOP_COUNT_MASK) >> FLAG_HOP_COUNT_SHIFT)

// Set frame type in flags
#define AGRINET_SET_FRAME_TYPE(flags, type) \
    (((flags) & ~FLAG_TYPE_MASK) | (((type) << FLAG_TYPE_SHIFT) & FLAG_TYPE_MASK))

#ifdef __cplusplus
}
#endif

#endif // AGRINET_PROTOCOL_H
