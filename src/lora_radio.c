/**
 * @file lora_radio.c
 * @brief SX1276 LoRa Radio Driver - register-level for STM32F103C8
 *
 * Ported from AGRINET Weather Station (ws-cld-ai-v3/firmware/field-node/src/lora_radio.c)
 * Adapted for master gateway pin mapping and FreeRTOS integration.
 */

#include "lora_radio.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

/* ═══════════════════════════════════════════════════════════════════════════
 * SX1276 REGISTERS
 * ═══════════════════════════════════════════════════════════════════════════ */
#define REG_FIFO                    0x00
#define REG_OP_MODE                 0x01
#define REG_FRF_MSB                 0x06
#define REG_FRF_MID                 0x07
#define REG_FRF_LSB                 0x08
#define REG_PA_CONFIG               0x09
#define REG_OCP                     0x0B
#define REG_LNA                     0x0C
#define REG_FIFO_ADDR_PTR           0x0D
#define REG_FIFO_TX_BASE_ADDR       0x0E
#define REG_FIFO_RX_BASE_ADDR       0x0F
#define REG_FIFO_RX_CURRENT_ADDR    0x10
#define REG_IRQ_FLAGS               0x12
#define REG_RX_NB_BYTES             0x13
#define REG_PKT_SNR_VALUE           0x19
#define REG_PKT_RSSI_VALUE          0x1A
#define REG_MODEM_CONFIG_1          0x1D
#define REG_MODEM_CONFIG_2          0x1E
#define REG_PREAMBLE_MSB            0x20
#define REG_PREAMBLE_LSB            0x21
#define REG_PAYLOAD_LENGTH          0x22
#define REG_MODEM_CONFIG_3          0x26
#define REG_DETECTION_OPTIMIZE      0x31
#define REG_DETECTION_THRESHOLD     0x37
#define REG_SYNC_WORD               0x39
#define REG_DIO_MAPPING_1           0x40
#define REG_VERSION                 0x42

/* Modes */
#define MODE_LONG_RANGE             0x80
#define MODE_SLEEP                  0x00
#define MODE_STDBY                  0x01
#define MODE_TX                     0x03
#define MODE_RX_CONTINUOUS          0x05

/* IRQ Flags */
#define IRQ_TX_DONE                 0x08
#define IRQ_RX_DONE                 0x40
#define IRQ_CRC_ERROR               0x20

/* PA */
#define PA_BOOST                    0x80

/* ═══════════════════════════════════════════════════════════════════════════
 * PRIVATE DATA
 * ═══════════════════════════════════════════════════════════════════════════ */

static SPI_HandleTypeDef s_hspi1;
static SemaphoreHandle_t s_spiMutex;
static StaticSemaphore_t s_spiMutexBuf;

static volatile bool s_txDone  = false;
static volatile bool s_rxDone  = false;
static volatile bool s_rxError = false;
static int16_t s_lastRSSI = 0;
static int8_t  s_lastSNR  = 0;

/* ═══════════════════════════════════════════════════════════════════════════
 * SPI COMMUNICATION
 * ═══════════════════════════════════════════════════════════════════════════ */

static void SPI_WriteByte(uint8_t reg, uint8_t val)
{
    xSemaphoreTake(s_spiMutex, portMAX_DELAY);
    HAL_GPIO_WritePin(PIN_LORA_NSS_PORT, PIN_LORA_NSS_PIN, GPIO_PIN_RESET);
    uint8_t addr = reg | 0x80;
    HAL_SPI_Transmit(&s_hspi1, &addr, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&s_hspi1, &val, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(PIN_LORA_NSS_PORT, PIN_LORA_NSS_PIN, GPIO_PIN_SET);
    xSemaphoreGive(s_spiMutex);
}

static uint8_t SPI_ReadByte(uint8_t reg)
{
    uint8_t val;
    xSemaphoreTake(s_spiMutex, portMAX_DELAY);
    HAL_GPIO_WritePin(PIN_LORA_NSS_PORT, PIN_LORA_NSS_PIN, GPIO_PIN_RESET);
    uint8_t addr = reg & 0x7F;
    HAL_SPI_Transmit(&s_hspi1, &addr, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&s_hspi1, &val, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(PIN_LORA_NSS_PORT, PIN_LORA_NSS_PIN, GPIO_PIN_SET);
    xSemaphoreGive(s_spiMutex);
    return val;
}

static void SPI_WriteBurst(uint8_t reg, const uint8_t* data, uint8_t len)
{
    xSemaphoreTake(s_spiMutex, portMAX_DELAY);
    HAL_GPIO_WritePin(PIN_LORA_NSS_PORT, PIN_LORA_NSS_PIN, GPIO_PIN_RESET);
    uint8_t addr = reg | 0x80;
    HAL_SPI_Transmit(&s_hspi1, &addr, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&s_hspi1, (uint8_t*)data, len, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(PIN_LORA_NSS_PORT, PIN_LORA_NSS_PIN, GPIO_PIN_SET);
    xSemaphoreGive(s_spiMutex);
}

static void SPI_ReadBurst(uint8_t reg, uint8_t* data, uint8_t len)
{
    xSemaphoreTake(s_spiMutex, portMAX_DELAY);
    HAL_GPIO_WritePin(PIN_LORA_NSS_PORT, PIN_LORA_NSS_PIN, GPIO_PIN_RESET);
    uint8_t addr = reg & 0x7F;
    HAL_SPI_Transmit(&s_hspi1, &addr, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&s_hspi1, data, len, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(PIN_LORA_NSS_PORT, PIN_LORA_NSS_PIN, GPIO_PIN_SET);
    xSemaphoreGive(s_spiMutex);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * HELPERS
 * ═══════════════════════════════════════════════════════════════════════════ */

static void SetMode(uint8_t mode) {
    SPI_WriteByte(REG_OP_MODE, MODE_LONG_RANGE | mode);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * PUBLIC API
 * ═══════════════════════════════════════════════════════════════════════════ */

bool LoRa_Init(void)
{
    s_spiMutex = xSemaphoreCreateMutexStatic(&s_spiMutexBuf);

    /* Enable clocks */
    __HAL_RCC_SPI1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* SPI1 pins: SCK(PA5), MISO(PA6), MOSI(PA7) */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = PIN_LORA_SCK_PIN | PIN_LORA_MOSI_PIN;
    gpio.Mode  = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin  = PIN_LORA_MISO_PIN;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* NSS (PA4) - manual control */
    gpio.Pin   = PIN_LORA_NSS_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(PIN_LORA_NSS_PORT, &gpio);
    HAL_GPIO_WritePin(PIN_LORA_NSS_PORT, PIN_LORA_NSS_PIN, GPIO_PIN_SET);

    /* RST (PB13) */
    gpio.Pin = PIN_LORA_RST_PIN;
    HAL_GPIO_Init(PIN_LORA_RST_PORT, &gpio);

    /* DIO0 (PB12) - interrupt input */
    gpio.Pin  = PIN_LORA_DIO0_PIN;
    gpio.Mode = GPIO_MODE_IT_RISING;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(PIN_LORA_DIO0_PORT, &gpio);
    HAL_NVIC_SetPriority(PIN_LORA_DIO0_IRQ, 5, 0);
    HAL_NVIC_EnableIRQ(PIN_LORA_DIO0_IRQ);

    /* Init SPI1 */
    s_hspi1.Instance               = SPI1;
    s_hspi1.Init.Mode              = SPI_MODE_MASTER;
    s_hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
    s_hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
    s_hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;
    s_hspi1.Init.CLKPhase          = SPI_PHASE_1EDGE;
    s_hspi1.Init.NSS               = SPI_NSS_SOFT;
    s_hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    s_hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    s_hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    if (HAL_SPI_Init(&s_hspi1) != HAL_OK) return false;

    /* Reset radio */
    HAL_GPIO_WritePin(PIN_LORA_RST_PORT, PIN_LORA_RST_PIN, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(PIN_LORA_RST_PORT, PIN_LORA_RST_PIN, GPIO_PIN_SET);
    HAL_Delay(10);

    /* Verify SX1276 */
    if (SPI_ReadByte(REG_VERSION) != 0x12) return false;

    /* Configure */
    SetMode(MODE_SLEEP);

    /* Frequency: 866 MHz (IN865 center) */
    uint64_t frf = ((uint64_t)LORA_FREQUENCY_DEFAULT << 19) / 32000000;
    SPI_WriteByte(REG_FRF_MSB, (uint8_t)(frf >> 16));
    SPI_WriteByte(REG_FRF_MID, (uint8_t)(frf >> 8));
    SPI_WriteByte(REG_FRF_LSB, (uint8_t)(frf));

    /* SF12, BW125, CR4/5 */
    uint8_t c1 = SPI_ReadByte(REG_MODEM_CONFIG_1);
    c1 = (c1 & 0x01) | (7 << 4) | (1 << 1);  // BW=125kHz, CR=4/5
    SPI_WriteByte(REG_MODEM_CONFIG_1, c1);

    uint8_t c2 = SPI_ReadByte(REG_MODEM_CONFIG_2);
    c2 = (c2 & 0x03) | (LORA_SPREADING_FACTOR << 4) | 0x04;  // SF12 + CRC on
    SPI_WriteByte(REG_MODEM_CONFIG_2, c2);

    /* Detection optimize for SF >= 7 */
    SPI_WriteByte(REG_DETECTION_OPTIMIZE, 0xC3);
    SPI_WriteByte(REG_DETECTION_THRESHOLD, 0x0A);

    /* TX power: 20 dBm with PA_BOOST */
    SPI_WriteByte(REG_PA_CONFIG, PA_BOOST | (LORA_TX_POWER - 2));

    /* Sync word, preamble */
    SPI_WriteByte(REG_SYNC_WORD, LORA_SYNC_WORD);
    SPI_WriteByte(REG_PREAMBLE_MSB, 0);
    SPI_WriteByte(REG_PREAMBLE_LSB, LORA_PREAMBLE_LENGTH);

    /* FIFO base addresses */
    SPI_WriteByte(REG_FIFO_TX_BASE_ADDR, 0x00);
    SPI_WriteByte(REG_FIFO_RX_BASE_ADDR, 0x00);

    /* DIO0 = TX Done / RX Done */
    SPI_WriteByte(REG_DIO_MAPPING_1, 0x00);

    /* LNA boost + AGC auto */
    SPI_WriteByte(REG_LNA, 0x23);
    uint8_t c3 = SPI_ReadByte(REG_MODEM_CONFIG_3);
    SPI_WriteByte(REG_MODEM_CONFIG_3, c3 | 0x04);

    SetMode(MODE_STDBY);
    return true;
}

void LoRa_SetFrequency(uint32_t freq)
{
    SetMode(MODE_STDBY);
    uint64_t frf = ((uint64_t)freq << 19) / 32000000;
    SPI_WriteByte(REG_FRF_MSB, (uint8_t)(frf >> 16));
    SPI_WriteByte(REG_FRF_MID, (uint8_t)(frf >> 8));
    SPI_WriteByte(REG_FRF_LSB, (uint8_t)(frf));
}

bool LoRa_Send(const uint8_t* data, uint8_t length)
{
    if (length == 0) return false;

    SetMode(MODE_STDBY);
    SPI_WriteByte(REG_FIFO_ADDR_PTR, 0);
    SPI_WriteBurst(REG_FIFO, data, length);
    SPI_WriteByte(REG_PAYLOAD_LENGTH, length);
    SPI_WriteByte(REG_IRQ_FLAGS, 0xFF);

    s_txDone = false;
    SetMode(MODE_TX);

    /* Wait for TX complete (max 5s for SF12) */
    uint32_t start = HAL_GetTick();
    while (!s_txDone) {
        if ((HAL_GetTick() - start) > 5000) {
            SetMode(MODE_STDBY);
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    SetMode(MODE_STDBY);
    return true;
}

void LoRa_StartReceive(void)
{
    SPI_WriteByte(REG_FIFO_ADDR_PTR, 0);
    SPI_WriteByte(REG_IRQ_FLAGS, 0xFF);
    s_rxDone = false;
    s_rxError = false;
    SetMode(MODE_RX_CONTINUOUS);
}

uint8_t LoRa_Receive(uint8_t* buf, uint8_t maxLen)
{
    if (!s_rxDone) return 0;
    s_rxDone = false;

    if (s_rxError) {
        LoRa_StartReceive();
        return 0;
    }

    /* Get RSSI and SNR */
    s_lastRSSI = SPI_ReadByte(REG_PKT_RSSI_VALUE) - 137;
    s_lastSNR  = (int8_t)SPI_ReadByte(REG_PKT_SNR_VALUE) / 4;

    /* Get payload length */
    uint8_t len = SPI_ReadByte(REG_RX_NB_BYTES);
    if (len > maxLen) len = maxLen;

    /* Read FIFO */
    SPI_WriteByte(REG_FIFO_ADDR_PTR, SPI_ReadByte(REG_FIFO_RX_CURRENT_ADDR));
    SPI_ReadBurst(REG_FIFO, buf, len);

    LoRa_StartReceive();
    return len;
}

void LoRa_Sleep(void) { SetMode(MODE_SLEEP); }
void LoRa_Wake(void)  { SetMode(MODE_STDBY); }
int16_t LoRa_GetRSSI(void) { return s_lastRSSI; }
int8_t  LoRa_GetSNR(void)  { return s_lastSNR; }

void LoRa_HandleDIO0_IRQ(void)
{
    uint8_t irq = SPI_ReadByte(REG_IRQ_FLAGS);
    if (irq & IRQ_TX_DONE) s_txDone = true;
    if (irq & IRQ_RX_DONE) {
        s_rxDone = true;
        s_rxError = (irq & IRQ_CRC_ERROR) != 0;
    }
    SPI_WriteByte(REG_IRQ_FLAGS, 0xFF);
}
