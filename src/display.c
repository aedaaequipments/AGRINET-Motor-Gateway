/**
 * @file display.c
 * @brief SSD1306 128x64 OLED - page-mode rendering (saves 1KB vs framebuffer)
 *
 * Uses I2C1 (PB6=SDA, PB7=SCL). Renders text using a compact 5x7 font.
 * Each "page" of the display is 128x8 pixels (8 pages for 64 rows).
 */

#include "display.h"
#include "power_monitor.h"
#include "motor_control.h"
#include "flash_config.h"
#include "gsm_driver.h"
#include "node_registry.h"
#include "automation.h"
#include <string.h>
#include <stdio.h>

/* ═══════════════════════════════════════════════════════════════════════════
 * SSD1306 COMMANDS
 * ═══════════════════════════════════════════════════════════════════════════ */
#define SSD1306_ADDR            (OLED_I2C_ADDR << 1)
#define SSD1306_CMD             0x00
#define SSD1306_DATA            0x40
#define SSD1306_DISPLAY_OFF     0xAE
#define SSD1306_DISPLAY_ON      0xAF
#define SSD1306_SET_MUX         0xA8
#define SSD1306_SET_OFFSET      0xD3
#define SSD1306_SET_START_LINE  0x40
#define SSD1306_SEG_REMAP       0xA1
#define SSD1306_COM_SCAN_DEC    0xC8
#define SSD1306_SET_COM_PINS    0xDA
#define SSD1306_SET_CONTRAST    0x81
#define SSD1306_ENTIRE_ON_RES   0xA4
#define SSD1306_NORMAL_DISPLAY  0xA6
#define SSD1306_SET_CLK_DIV     0xD5
#define SSD1306_CHARGE_PUMP     0x8D
#define SSD1306_SET_ADDR_MODE   0x20
#define SSD1306_SET_COL_ADDR    0x21
#define SSD1306_SET_PAGE_ADDR   0x22

/* ═══════════════════════════════════════════════════════════════════════════
 * PRIVATE DATA
 * ═══════════════════════════════════════════════════════════════════════════ */

static I2C_HandleTypeDef s_hi2c1;
static DisplayPage_t s_currentPage = PAGE_POWER;
static uint32_t s_pageChangeTick = 0;
static bool s_initialized = false;

/* Line buffer for text rendering (128 bytes = one row of 8px height) */
static uint8_t s_lineBuf[128];

/* ═══════════════════════════════════════════════════════════════════════════
 * COMPACT 5x7 FONT (ASCII 32-127, stored as 5 bytes per character)
 * Only include printable characters to save flash
 * ═══════════════════════════════════════════════════════════════════════════ */

static const uint8_t font5x7[] = {
    0x00,0x00,0x00,0x00,0x00, // Space
    0x00,0x00,0x5F,0x00,0x00, // !
    0x00,0x07,0x00,0x07,0x00, // "
    0x14,0x7F,0x14,0x7F,0x14, // #
    0x24,0x2A,0x7F,0x2A,0x12, // $
    0x23,0x13,0x08,0x64,0x62, // %
    0x36,0x49,0x56,0x20,0x50, // &
    0x00,0x08,0x07,0x03,0x00, // '
    0x00,0x1C,0x22,0x41,0x00, // (
    0x00,0x41,0x22,0x1C,0x00, // )
    0x2A,0x1C,0x7F,0x1C,0x2A, // *
    0x08,0x08,0x3E,0x08,0x08, // +
    0x00,0x80,0x70,0x30,0x00, // ,
    0x08,0x08,0x08,0x08,0x08, // -
    0x00,0x00,0x60,0x60,0x00, // .
    0x20,0x10,0x08,0x04,0x02, // /
    0x3E,0x51,0x49,0x45,0x3E, // 0
    0x00,0x42,0x7F,0x40,0x00, // 1
    0x72,0x49,0x49,0x49,0x46, // 2
    0x21,0x41,0x49,0x4D,0x33, // 3
    0x18,0x14,0x12,0x7F,0x10, // 4
    0x27,0x45,0x45,0x45,0x39, // 5
    0x3C,0x4A,0x49,0x49,0x31, // 6
    0x41,0x21,0x11,0x09,0x07, // 7
    0x36,0x49,0x49,0x49,0x36, // 8
    0x46,0x49,0x49,0x29,0x1E, // 9
    0x00,0x00,0x14,0x00,0x00, // :
    0x00,0x40,0x34,0x00,0x00, // ;
    0x00,0x08,0x14,0x22,0x41, // <
    0x14,0x14,0x14,0x14,0x14, // =
    0x00,0x41,0x22,0x14,0x08, // >
    0x02,0x01,0x59,0x09,0x06, // ?
    0x3E,0x41,0x5D,0x59,0x4E, // @
    0x7C,0x12,0x11,0x12,0x7C, // A
    0x7F,0x49,0x49,0x49,0x36, // B
    0x3E,0x41,0x41,0x41,0x22, // C
    0x7F,0x41,0x41,0x41,0x3E, // D
    0x7F,0x49,0x49,0x49,0x41, // E
    0x7F,0x09,0x09,0x09,0x01, // F
    0x3E,0x41,0x41,0x51,0x73, // G
    0x7F,0x08,0x08,0x08,0x7F, // H
    0x00,0x41,0x7F,0x41,0x00, // I
    0x20,0x40,0x41,0x3F,0x01, // J
    0x7F,0x08,0x14,0x22,0x41, // K
    0x7F,0x40,0x40,0x40,0x40, // L
    0x7F,0x02,0x1C,0x02,0x7F, // M
    0x7F,0x04,0x08,0x10,0x7F, // N
    0x3E,0x41,0x41,0x41,0x3E, // O
    0x7F,0x09,0x09,0x09,0x06, // P
    0x3E,0x41,0x51,0x21,0x5E, // Q
    0x7F,0x09,0x19,0x29,0x46, // R
    0x26,0x49,0x49,0x49,0x32, // S
    0x03,0x01,0x7F,0x01,0x03, // T
    0x3F,0x40,0x40,0x40,0x3F, // U
    0x1F,0x20,0x40,0x20,0x1F, // V
    0x3F,0x40,0x38,0x40,0x3F, // W
    0x63,0x14,0x08,0x14,0x63, // X
    0x03,0x04,0x78,0x04,0x03, // Y
    0x61,0x59,0x49,0x4D,0x43, // Z
};

/* ═══════════════════════════════════════════════════════════════════════════
 * LOW-LEVEL I2C
 * ═══════════════════════════════════════════════════════════════════════════ */

static void WriteCmd(uint8_t cmd)
{
    uint8_t buf[2] = { SSD1306_CMD, cmd };
    HAL_I2C_Master_Transmit(&s_hi2c1, SSD1306_ADDR, buf, 2, 100);
}

static void WriteData(const uint8_t* data, uint16_t len)
{
    /* Send in chunks with data prefix */
    uint8_t buf[129];  // 1 prefix + 128 data max
    buf[0] = SSD1306_DATA;
    uint16_t chunk = (len > 128) ? 128 : len;
    memcpy(buf + 1, data, chunk);
    HAL_I2C_Master_Transmit(&s_hi2c1, SSD1306_ADDR, buf, chunk + 1, 200);
}

/**
 * M1 FIX: SH1106 uses page-mode addressing (not horizontal mode like SSD1306).
 * SH1106 has 132-column RAM with 128 visible, offset by +2 columns.
 * Use page address + column high/low nibble commands.
 */
static void SetCursor(uint8_t page, uint8_t col)
{
    col += 2;  /* SH1106 column offset: 132-col RAM, 128 visible, +2 offset */
    WriteCmd(0xB0 | (page & 0x07));           /* Set page address (0xB0-0xB7) */
    WriteCmd(0x00 | (col & 0x0F));            /* Set lower column nibble */
    WriteCmd(0x10 | ((col >> 4) & 0x0F));     /* Set upper column nibble */
}

static void ClearScreen(void)
{
    memset(s_lineBuf, 0, 128);
    for (uint8_t p = 0; p < 8; p++) {
        SetCursor(p, 0);
        WriteData(s_lineBuf, 128);
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * TEXT RENDERING (5x7 font, 21 chars per line, 8 lines)
 * ═══════════════════════════════════════════════════════════════════════════ */

static void RenderLine(uint8_t page, const char* text)
{
    memset(s_lineBuf, 0, 128);
    uint8_t col = 0;

    for (uint8_t i = 0; text[i] && col < 123; i++) {
        uint8_t ch = (uint8_t)text[i];
        if (ch < 32 || ch > 90) ch = 32;  // Clamp to supported range

        const uint8_t* glyph = &font5x7[(ch - 32) * 5];
        for (uint8_t j = 0; j < 5; j++) {
            s_lineBuf[col++] = glyph[j];
        }
        s_lineBuf[col++] = 0x00;  // 1px spacing
    }

    SetCursor(page, 0);
    WriteData(s_lineBuf, 128);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * PAGE RENDERERS
 * ═══════════════════════════════════════════════════════════════════════════ */

static char s_line[22];  // 21 chars + null

static void RenderPowerPage(void)
{
    PowerSnapshot_t snap;
    PowerMonitor_GetSnapshot(&snap);

    RenderLine(0, "=== POWER ===");
    snprintf(s_line, 22, "R: %3.0fV %4.1fA", snap.r.voltage, snap.r.current);
    RenderLine(1, s_line);
    snprintf(s_line, 22, "Y: %3.0fV %4.1fA", snap.y.voltage, snap.y.current);
    RenderLine(2, s_line);
    snprintf(s_line, 22, "B: %3.0fV %4.1fA", snap.b.voltage, snap.b.current);
    RenderLine(3, s_line);
    snprintf(s_line, 22, "PF: %.2f", snap.avgPF);
    RenderLine(5, s_line);
    snprintf(s_line, 22, "TOTAL: %.2f KW", snap.totalPower);
    RenderLine(6, s_line);
    snprintf(s_line, 22, "IMBAL: %.0f%%", snap.currentImbalance);
    RenderLine(7, s_line);
}

static void RenderMotorPage(void)
{
    FlashConfig_t* cfg = FlashConfig_Get();
    FaultCode_t fault = MotorControl_GetFault();

    RenderLine(0, "=== MOTOR ===");
    snprintf(s_line, 22, "ID: %s", cfg->deviceId);
    RenderLine(1, s_line);

    const char* stateStr = "IDLE";
    switch (MotorControl_GetState()) {
        case MOTOR_STAR:  stateStr = "STAR"; break;
        case MOTOR_DELTA: stateStr = "RUNNING"; break;
        case MOTOR_FAULT: stateStr = "FAULT"; break;
        case MOTOR_STAR_TO_DELTA: stateStr = "S->D"; break;
        default: break;
    }
    snprintf(s_line, 22, "STATE: %s", stateStr);
    RenderLine(2, s_line);

    if (cfg->calibration.isCalibrated) {
        snprintf(s_line, 22, "HP: %.1f  I: %.1fA", cfg->calibration.ratedHP, cfg->calibration.ratedCurrent);
    } else {
        snprintf(s_line, 22, "HP: NOT CALIBRATED");
    }
    RenderLine(3, s_line);

    snprintf(s_line, 22, "SD: %.1fS", (float)cfg->starDeltaDelay / 10.0f);
    RenderLine(4, s_line);

    snprintf(s_line, 22, "MODE: %s%s%s",
        cfg->mode == MODE_AUTO ? "AUTO" : "MAN",
        cfg->safeMode ? " SAFE" : "",
        cfg->forceRun ? " FORCE" : "");
    RenderLine(5, s_line);

    snprintf(s_line, 22, "KWH:%.0f HRS:%.0f", cfg->stats.totalKwh, cfg->stats.totalHours);
    RenderLine(6, s_line);

    if (fault != FAULT_NONE) {
        snprintf(s_line, 22, "FAULT: %s", MotorControl_FaultString(fault));
        RenderLine(7, s_line);
    } else {
        RenderLine(7, "FAULT: NONE");
    }
}

static void RenderWeatherPage(void)
{
    WeatherPayload_t w;
    uint64_t uid;

    RenderLine(0, "=== WEATHER ===");

    if (!LoRaManager_GetLastWeatherData(&w, &uid)) {
        RenderLine(2, "NO DATA");
        RenderLine(3, "WAITING FOR WS...");
        return;
    }

    snprintf(s_line, 22, "AIR: %.1fC  H:%.0f%%", (float)w.airTemp/100.0f, (float)w.humidity/100.0f);
    RenderLine(1, s_line);
    snprintf(s_line, 22, "SOIL: %.1fC", (float)w.soilTemp/100.0f);
    RenderLine(2, s_line);
    snprintf(s_line, 22, "L1:%.0f L2:%.0f L3:%.0f", (float)w.soilM1/100.0f, (float)w.soilM2/100.0f, (float)w.soilM3/100.0f);
    RenderLine(3, s_line);
    snprintf(s_line, 22, "LEAF: %u%%  %u%%", w.leafWet1, w.leafWet2);
    RenderLine(4, s_line);
    snprintf(s_line, 22, "WIND: %.1fM/S", (float)w.windSpeed/100.0f);
    RenderLine(5, s_line);
    snprintf(s_line, 22, "RAIN: %.1fMM", (float)w.rainMm/10.0f);
    RenderLine(6, s_line);

    RiskAssessment_t risk;
    Automation_GetRisk(&risk);
    snprintf(s_line, 22, "PEST:%u%% DIS:%u%%", risk.pestRisk, risk.diseaseRisk);
    RenderLine(7, s_line);
}

static void RenderNetworkPage(void)
{
    RenderLine(0, "=== NETWORK ===");

    snprintf(s_line, 22, "GSM: %s SIG:%u",
        GSM_IsReady() ? "OK" : "DOWN", GSM_GetSignalQuality());
    RenderLine(1, s_line);

    snprintf(s_line, 22, "LORA: %u NODES", NodeRegistry_OnlineCount());
    RenderLine(2, s_line);

    /* List nodes */
    uint8_t count;
    const NodeEntry_t* nodes = NodeRegistry_GetAll(&count);
    uint8_t line = 3;
    for (uint8_t i = 0; i < MAX_NODES && line < 8; i++) {
        if (nodes[i].registered) {
            snprintf(s_line, 22, " %c %s %ddBm",
                nodes[i].type,
                nodes[i].online ? "ON " : "OFF",
                nodes[i].rssi);
            RenderLine(line++, s_line);
        }
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * PUBLIC API
 * ═══════════════════════════════════════════════════════════════════════════ */

bool Display_Init(void)
{
    __HAL_RCC_I2C1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = PIN_OLED_SDA_PIN | PIN_OLED_SCL_PIN;
    gpio.Mode  = GPIO_MODE_AF_OD;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);

    s_hi2c1.Instance             = I2C1;
    s_hi2c1.Init.ClockSpeed      = 400000;
    s_hi2c1.Init.DutyCycle        = I2C_DUTYCYCLE_2;
    s_hi2c1.Init.OwnAddress1     = 0;
    s_hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    if (HAL_I2C_Init(&s_hi2c1) != HAL_OK) return false;

    /* Check if OLED is present */
    if (HAL_I2C_IsDeviceReady(&s_hi2c1, SSD1306_ADDR, 3, 100) != HAL_OK) {
        return false;
    }

    /* M1 FIX: Init sequence for SH1106 (1.3" OLED) — page-mode only.
     * SH1106 does NOT support horizontal addressing mode (0x20 cmd).
     * Uses page addressing with column set via high/low nibble commands. */
    WriteCmd(SSD1306_DISPLAY_OFF);
    WriteCmd(SSD1306_SET_MUX);        WriteCmd(63);         /* 64 rows */
    WriteCmd(SSD1306_SET_OFFSET);     WriteCmd(0);
    WriteCmd(SSD1306_SET_START_LINE);                        /* Start line 0 */
    WriteCmd(SSD1306_SEG_REMAP);                             /* Col 127 = SEG0 */
    WriteCmd(SSD1306_COM_SCAN_DEC);                          /* COM63 to COM0 */
    WriteCmd(SSD1306_SET_COM_PINS);   WriteCmd(0x12);        /* Alt COM, no remap */
    WriteCmd(SSD1306_SET_CONTRAST);   WriteCmd(0x80);        /* SH1106 default */
    WriteCmd(SSD1306_ENTIRE_ON_RES);
    WriteCmd(SSD1306_NORMAL_DISPLAY);
    WriteCmd(SSD1306_SET_CLK_DIV);    WriteCmd(0x80);
    WriteCmd(SSD1306_CHARGE_PUMP);    WriteCmd(0x14);        /* Enable charge pump */
    /* NOTE: SSD1306_SET_ADDR_MODE removed — SH1106 only supports page mode */
    WriteCmd(SSD1306_DISPLAY_ON);

    ClearScreen();
    s_initialized = true;
    s_pageChangeTick = HAL_GetTick();
    return true;
}

void Display_Update(void)
{
    if (!s_initialized) return;

    /* Auto-cycle pages every 3 seconds */
    if ((HAL_GetTick() - s_pageChangeTick) > 3000) {
        s_pageChangeTick = HAL_GetTick();
        s_currentPage = (DisplayPage_t)((s_currentPage + 1) % PAGE_COUNT);
        ClearScreen();
    }

    switch (s_currentPage) {
        case PAGE_POWER:   RenderPowerPage();   break;
        case PAGE_MOTOR:   RenderMotorPage();   break;
        case PAGE_WEATHER: RenderWeatherPage(); break;
        case PAGE_VALVES:  /* Placeholder */     break;
        case PAGE_NETWORK: RenderNetworkPage(); break;
        default: break;
    }
}

void Display_NextPage(void)
{
    s_currentPage = (DisplayPage_t)((s_currentPage + 1) % PAGE_COUNT);
    s_pageChangeTick = HAL_GetTick();
    if (s_initialized) ClearScreen();
}

void Display_SetPage(DisplayPage_t page) { s_currentPage = page; }
DisplayPage_t Display_GetPage(void) { return s_currentPage; }

void Display_Splash(void)
{
    if (!s_initialized) return;
    ClearScreen();
    RenderLine(2, "   AGRINET V" FW_VERSION_STR);
    RenderLine(3, "  MASTER GATEWAY");
    RenderLine(5, " SASYAMITHRA IOT");
}

void Display_Error(const char* line1, const char* line2)
{
    if (!s_initialized) return;
    ClearScreen();
    RenderLine(2, "!!! ERROR !!!");
    if (line1) RenderLine(4, line1);
    if (line2) RenderLine(5, line2);
}
