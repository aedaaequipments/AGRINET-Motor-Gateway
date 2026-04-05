/**
 * @file lora_radio.h
 * @brief SX1276 LoRa Radio Driver (register-level, no library)
 *
 * Ported from AGRINET Weather Station driver.
 * Uses SPI1 (PA4-PA7), DIO0 on PB12, RST on PB13.
 * IN865 band (865-867 MHz, WPC compliant).
 */

#ifndef LORA_RADIO_H
#define LORA_RADIO_H

#include "config.h"
#include "agrinet_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize SX1276 radio on SPI1
 * @return true if SX1276 detected and configured
 */
bool LoRa_Init(void);

/**
 * @brief Send raw data via LoRa
 * @param data Buffer to send
 * @param length Length in bytes (max 255)
 * @return true on successful transmission
 */
bool LoRa_Send(const uint8_t* data, uint8_t length);

/**
 * @brief Start continuous receive mode
 */
void LoRa_StartReceive(void);

/**
 * @brief Check if a packet has been received
 * @param buf Buffer to store received data
 * @param maxLen Maximum buffer length
 * @return Number of bytes received, 0 if none
 */
uint8_t LoRa_Receive(uint8_t* buf, uint8_t maxLen);

/**
 * @brief Set operating frequency
 * @param freq Frequency in Hz (e.g., 866000000)
 */
void LoRa_SetFrequency(uint32_t freq);

/**
 * @brief Put radio to sleep (low power)
 */
void LoRa_Sleep(void);

/**
 * @brief Wake radio to standby
 */
void LoRa_Wake(void);

/**
 * @brief Get RSSI of last received packet
 * @return RSSI in dBm
 */
int16_t LoRa_GetRSSI(void);

/**
 * @brief Get SNR of last received packet
 * @return SNR in dB
 */
int8_t LoRa_GetSNR(void);

/**
 * @brief Check if TX/RX interrupt fired (call from DIO0 EXTI handler)
 */
void LoRa_HandleDIO0_IRQ(void);

#ifdef __cplusplus
}
#endif

#endif /* LORA_RADIO_H */
