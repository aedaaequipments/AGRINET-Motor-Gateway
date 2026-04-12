/**
 * @file display.c
 * @brief SH1106 128x64 OLED - page-mode rendering (saves 1KB vs framebuffer)
 *
 * Uses I2C1 (PB6=SCL, PB7=SDA). Renders text using a compact 5x7 font.
 * Each "page" of the display is 128x8 pixels (8 pages for 64 rows).
 * SH1106 has 132-column internal RAM; visible columns start at offset 2.
 */

#include "display.h"
#include "power_monitor.h"
#include "motor_control.h"
#include "flash_config.h"
#include "gsm_driver.h"
#include "lora_manager.h"
#include "node_registry.h"
#include "automation.h"
#include "offline_queue.h"
#include <string.h>
#include <stdio.h>

/* ═══════════════════════════════════════════════════════════════════════════
 * SH1106 COMMANDS (1.3" OLED, 132-column controller)
 * ═══════════════════════════════════════════════════════════════════════════ */
static uint8_t s_oled_i2c_addr = OLED_I2C_ADDR;
#define SSD1306_ADDR            (s_oled_i2c_addr << 1)
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
/* SH1106 charge pump: 0xAD=enable cmd, 0x8B=on (different from SSD1306) */
#define SH1106_CHARGE_PUMP_ON   0xAD
#define SH1106_CHARGE_PUMP_VAL  0x8B
/* SH1106 column offset: internal RAM is 132 cols, display starts at col 2 */
#define SH1106_COL_OFFSET       2

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
 * COMPACT 5x7 FONT (ASCII 32-126, stored as 5 bytes per character)
 * Full printable ASCII range: space through tilde
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
    0x00,0x7F,0x41,0x41,0x41, // [
    0x02,0x04,0x08,0x10,0x20, // backslash
    0x41,0x41,0x41,0x7F,0x00, // ]
    0x04,0x02,0x01,0x02,0x04, // ^
    0x40,0x40,0x40,0x40,0x40, // _
    0x00,0x03,0x07,0x08,0x00, // `
    0x20,0x54,0x54,0x78,0x40, // a
    0x7F,0x28,0x44,0x44,0x38, // b
    0x38,0x44,0x44,0x44,0x28, // c
    0x38,0x44,0x44,0x28,0x7F, // d
    0x38,0x54,0x54,0x54,0x18, // e
    0x00,0x08,0x7E,0x09,0x02, // f
    0x18,0xA4,0xA4,0x9C,0x78, // g
    0x7F,0x08,0x04,0x04,0x78, // h
    0x00,0x44,0x7D,0x40,0x00, // i
    0x20,0x40,0x40,0x3D,0x00, // j
    0x7F,0x10,0x28,0x44,0x00, // k
    0x00,0x41,0x7F,0x40,0x00, // l
    0x7C,0x04,0x78,0x04,0x78, // m
    0x7C,0x08,0x04,0x04,0x78, // n
    0x38,0x44,0x44,0x44,0x38, // o
    0xFC,0x18,0x24,0x24,0x18, // p
    0x18,0x24,0x24,0x18,0xFC, // q
    0x7C,0x08,0x04,0x04,0x08, // r
    0x48,0x54,0x54,0x54,0x24, // s
    0x04,0x04,0x3F,0x44,0x24, // t
    0x3C,0x40,0x40,0x20,0x7C, // u
    0x1C,0x20,0x40,0x20,0x1C, // v
    0x3C,0x40,0x30,0x40,0x3C, // w
    0x44,0x28,0x10,0x28,0x44, // x
    0x4C,0x90,0x90,0x90,0x7C, // y
    0x44,0x64,0x54,0x4C,0x44, // z
    0x00,0x08,0x36,0x41,0x00, // {
    0x00,0x00,0x77,0x00,0x00, // |
    0x00,0x41,0x36,0x08,0x00, // }
    0x02,0x01,0x02,0x04,0x02, // ~
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
 * Set cursor using SH1106 page-mode addressing.
 * SH1106 has 132-column internal RAM; add SH1106_COL_OFFSET=2 to align
 * the 128 visible columns correctly.
 */
static void SetCursor(uint8_t page, uint8_t col)
{
    uint8_t c = col + SH1106_COL_OFFSET;
    WriteCmd(0xB0 | (page & 0x07));      /* Set page address */
    WriteCmd(0x00 | (c & 0x0F));         /* Set low column nibble */
    WriteCmd(0x10 | (c >> 4));           /* Set high column nibble */
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
 * TEXT RENDERING
 *   RenderLine()  — 5x7 small text, 21 chars/line, 1 page (8px)
 *   RenderBig()   — 10x14 doubled text, 10 chars/line, 2 pages (16px)
 * ═══════════════════════════════════════════════════════════════════════════ */

static void RenderLine(uint8_t page, const char* text)
{
    memset(s_lineBuf, 0, 128);
    uint8_t col = 0;

    for (uint8_t i = 0; text[i] && col < 123; i++) {
        uint8_t ch = (uint8_t)text[i];
        if (ch < 32 || ch > 126) ch = 32;

        const uint8_t* glyph = &font5x7[(ch - 32) * 5];
        for (uint8_t j = 0; j < 5; j++) {
            s_lineBuf[col++] = glyph[j];
        }
        s_lineBuf[col++] = 0x00;
    }

    SetCursor(page, 0);
    WriteData(s_lineBuf, 128);
}

/**
 * Render 2x scaled text — each glyph column is doubled horizontally
 * and each bit is stretched to 2 vertical bits (spans 2 pages = 16px).
 * Fits ~10 chars per line. Uses pages [page] and [page+1].
 */
static uint8_t s_bigBufTop[128];
static uint8_t s_bigBufBot[128];

static void RenderBig(uint8_t page, const char* text)
{
    memset(s_bigBufTop, 0, 128);
    memset(s_bigBufBot, 0, 128);
    uint8_t col = 0;

    for (uint8_t i = 0; text[i] && col < 120; i++) {
        uint8_t ch = (uint8_t)text[i];
        if (ch < 32 || ch > 126) ch = 32;

        const uint8_t* glyph = &font5x7[(ch - 32) * 5];
        for (uint8_t j = 0; j < 5; j++) {
            /* Stretch each bit vertically: 7 source bits → 14 dest bits
             * Top page gets bits 0-7, bottom page gets bits 0-7
             * Source bit N → dest bits 2N and 2N+1 */
            uint8_t src = glyph[j];
            uint16_t stretched = 0;
            for (uint8_t b = 0; b < 7; b++) {
                if (src & (1 << b)) {
                    stretched |= (3 << (b * 2));  /* 2 bits per source bit */
                }
            }
            uint8_t top = (uint8_t)(stretched & 0xFF);
            uint8_t bot = (uint8_t)((stretched >> 8) & 0xFF);

            /* Double each column horizontally */
            s_bigBufTop[col]   = top;
            s_bigBufTop[col+1] = top;
            s_bigBufBot[col]   = bot;
            s_bigBufBot[col+1] = bot;
            col += 2;
        }
        /* 2px spacing between chars */
        col += 2;
    }

    SetCursor(page, 0);
    WriteData(s_bigBufTop, 128);
    SetCursor(page + 1, 0);
    WriteData(s_bigBufBot, 128);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * PAGE RENDERERS
 * ═══════════════════════════════════════════════════════════════════════════ */

static char s_line[22];  // 21 chars + null

static void RenderPowerPage(void)
{
    PowerSnapshot_t snap;
    PowerMonitor_GetSnapshot(&snap);

    /* Row 0-1: Voltages BIG */
    snprintf(s_line, 22, "%d %d %dV",
        (int)snap.r.voltage, (int)snap.y.voltage, (int)snap.b.voltage);
    RenderBig(0, s_line);

    /* Row 2-3: Currents BIG */
    snprintf(s_line, 22, "%.1f %.1f %.1fA",
        snap.r.current, snap.y.current, snap.b.current);
    RenderBig(2, s_line);

    /* Row 4-5: Total Power BIG */
    snprintf(s_line, 22, "%.2f kW", snap.totalPower);
    RenderBig(4, s_line);

    /* Row 6-7: PF and Imbalance BIG */
    snprintf(s_line, 22, "PF%.2f I%d%%", snap.avgPF, (int)snap.currentImbalance);
    RenderBig(6, s_line);
}

static void RenderMotorPage(void)
{
    FlashConfig_t* cfg = FlashConfig_Get();
    FaultCode_t fault = MotorControl_GetFault();

    const char* stateStr = "STOP";
    switch (MotorControl_GetState()) {
        case MOTOR_STAR:          stateStr = "START"; break;
        case MOTOR_DELTA:         stateStr = "RUN"; break;
        case MOTOR_FAULT:         stateStr = "FAULT!"; break;
        case MOTOR_STAR_TO_DELTA: stateStr = "S->D"; break;
        default: break;
    }

    RenderBig(0, stateStr);

    snprintf(s_line, 22, "%s %.1fHP %s",
        cfg->mode == MODE_AUTO ? "AUTO" : "MAN",
        cfg->calibration.ratedHP,
        cfg->forceRun ? "FORCE!" : "");
    RenderLine(2, s_line);

    /* Limits display */
    snprintf(s_line, 22, "LIM:%.0fV-%.0fV %.0fA",
        cfg->protection.underVoltage,
        cfg->protection.overVoltage,
        cfg->protection.overloadAmps);
    RenderLine(3, s_line);

    snprintf(s_line, 22, "%.0fkWh %.0fH #%lu",
        cfg->stats.totalKwh, cfg->stats.totalHours,
        cfg->stats.startCount);
    RenderLine(4, s_line);

    if (fault != FAULT_NONE) {
        snprintf(s_line, 22, "!%s", MotorControl_FaultString(fault));
        RenderBig(5, s_line);
    }
    if (cfg->forceRun) {
        RenderLine(7, "!! FORCE RUN !!");
    }
}

static void RenderNetworkPage(void)
{
    uint8_t regCount;
    const NodeEntry_t* nodes = NodeRegistry_GetAll(&regCount);
    uint8_t totalReg = 0, totalOnline = 0;
    for (uint8_t i = 0; i < MAX_NODES; i++) {
        if (nodes[i].registered) {
            totalReg++;
            if (nodes[i].online) totalOnline++;
        }
    }

    /* Big cloud status */
    GsmState_t gsmState = GSM_GetState();
    if (gsmState == GSM_STATE_MQTT_CONNECTED) {
        RenderBig(0, "CLOUD OK");
    } else if (GSM_IsReady()) {
        RenderBig(0, "GSM OK");
    } else {
        RenderBig(0, "OFFLINE!");
    }

    snprintf(s_line, 22, "SIG:%u Q:%u", GSM_GetSignalQuality(), OfflineQueue_Count());
    RenderLine(2, s_line);

    snprintf(s_line, 22, "NODES %u/%u", totalOnline, totalReg);
    RenderLine(3, s_line);

    /* List nodes compact */
    uint8_t line = 4;
    for (uint8_t i = 0; i < MAX_NODES && line < 8; i++) {
        if (nodes[i].registered) {
            snprintf(s_line, 22, "%c%u:%s %ddB",
                nodes[i].type, i,
                nodes[i].online ? "OK" : "X",
                nodes[i].rssi);
            RenderLine(line++, s_line);
        }
    }
}

static void RenderWeatherPage(void)
{
    WeatherPayload_t w;
    uint64_t uid;

    if (!LoRaManager_GetLastWeatherData(&w, &uid)) {
        RenderBig(0, "NO WS");
        RenderLine(3, "WAITING...");
        return;
    }

    snprintf(s_line, 22, "%.1fC %.0f%%", (float)w.airTemp/100.0f, (float)w.humidity/100.0f);
    RenderBig(0, s_line);

    snprintf(s_line, 22, "M1:%.0f M2:%.0f M3:%.0f",
        (float)w.soilM1/100.0f, (float)w.soilM2/100.0f, (float)w.soilM3/100.0f);
    RenderLine(2, s_line);

    snprintf(s_line, 22, "SOIL:%.1fC LF:%u%%", (float)w.soilTemp/100.0f, w.leafWet1);
    RenderLine(3, s_line);

    snprintf(s_line, 22, "W:%.1fm/s R:%.1fmm", (float)w.windSpeed/100.0f, (float)w.rainMm/10.0f);
    RenderLine(4, s_line);

    RiskAssessment_t risk;
    Automation_GetRisk(&risk);
    snprintf(s_line, 22, "PST:%u%% DIS:%u%%", risk.pestRisk, risk.diseaseRisk);
    RenderLine(5, s_line);
}

static void RenderValvesPage(void)
{
    uint8_t count;
    const NodeEntry_t* nodes = NodeRegistry_GetAll(&count);

    RenderLine(0, "VALVES");

    uint8_t line = 1;
    bool anyValve = false;
    for (uint8_t i = 0; i < MAX_NODES && line < 7; i++) {
        if (nodes[i].registered && nodes[i].type == 'V') {
            anyValve = true;
            if (!nodes[i].online) {
                snprintf(s_line, 22, "V%u: OFFLINE!", i);
                RenderBig(line, s_line);
                line += 2;
            } else if (nodes[i].dataValid) {
                snprintf(s_line, 22, "V%u:%u%% %.1fL", i,
                    nodes[i].data.valve.position,
                    (float)nodes[i].data.valve.flowRate / 10.0f);
                RenderLine(line++, s_line);
            } else {
                snprintf(s_line, 22, "V%u:OK --", i);
                RenderLine(line++, s_line);
            }
        }
    }
    if (!anyValve) {
        RenderBig(2, "NO VALVES");
    }
}

static void RenderAlertsPage(void)
{
    uint8_t line = 0;
    bool hasAlert = false;
    PowerSnapshot_t snap;
    PowerMonitor_GetSnapshot(&snap);
    FlashConfig_t* cfg = FlashConfig_Get();

    /* Motor fault — BIG priority alert */
    FaultCode_t fault = MotorControl_GetFault();
    if (fault != FAULT_NONE && line < 6) {
        snprintf(s_line, 22, "!%s", MotorControl_FaultString(fault));
        RenderBig(line, s_line);
        line += 2;
        hasAlert = true;
    }

    /* Phase voltage analysis */
    if (snap.r.voltage > 50.0f) {
        float maxV = snap.r.voltage;
        float minV = snap.r.voltage;
        if (snap.y.voltage > maxV) maxV = snap.y.voltage;
        if (snap.b.voltage > maxV) maxV = snap.b.voltage;
        if (snap.y.voltage < minV) minV = snap.y.voltage;
        if (snap.b.voltage < minV) minV = snap.b.voltage;
        float vImbal = ((maxV - minV) / ((maxV + minV) / 2.0f)) * 100.0f;
        if (vImbal > 5.0f && line < 6) {
            snprintf(s_line, 22, "!V IMB %.0f%%", vImbal);
            RenderBig(line, s_line);
            line += 2;
            hasAlert = true;
        }
    }

    /* Current imbalance — BIG */
    if (snap.currentImbalance > 10.0f && line < 6) {
        snprintf(s_line, 22, "!I IMB %.0f%%", snap.currentImbalance);
        RenderBig(line, s_line);
        line += 2;
        hasAlert = true;
    }

    /* Low PF warning */
    if (snap.avgPF < 0.75f && snap.avgPF > 0.01f && line < 7) {
        snprintf(s_line, 22, "!PF LOW %.2f", snap.avgPF);
        RenderLine(line++, s_line);
        hasAlert = true;
    }

    /* Supply voltage out of range */
    if (cfg->calibration.isCalibrated && snap.avgVoltage > 50.0f && line < 7) {
        float vPct = snap.avgVoltage / cfg->calibration.ratedVoltage;
        if (vPct < 0.90f) {
            snprintf(s_line, 22, "!LOW SUPPLY %.0fV", snap.avgVoltage);
            RenderLine(line++, s_line);
            hasAlert = true;
        } else if (vPct > 1.10f) {
            snprintf(s_line, 22, "!HIGH SUPPLY %.0fV", snap.avgVoltage);
            RenderLine(line++, s_line);
            hasAlert = true;
        }
    }

    /* Cloud */
    if (!GSM_IsReady() && line < 7) {
        RenderLine(line++, "!NO CLOUD");
        hasAlert = true;
    }

    /* Force run warning */
    if (cfg->forceRun && line < 7) {
        RenderLine(line++, "!FORCE MODE ON");
        hasAlert = true;
    }

    if (!hasAlert) {
        RenderBig(3, "ALL OK");
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * PUBLIC API
 * ═══════════════════════════════════════════════════════════════════════════ */

bool Display_Init(void)
{
    __HAL_RCC_I2C1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* STM32F1 I2C BUSY flag errata workaround (AN2824):
     * After power-on or watchdog reset, BUSY can be stuck HIGH.
     * GD32 clones do NOT have this bug, but this workaround is safe for both. */

    /* First: de-init I2C peripheral so GPIO can be used freely */
    __HAL_RCC_I2C1_FORCE_RESET();
    HAL_Delay(1);
    __HAL_RCC_I2C1_RELEASE_RESET();

    /* Configure SDA as open-drain output to check/release bus */
    GPIO_InitTypeDef sda_gpio = {0};
    sda_gpio.Pin   = PIN_OLED_SDA_PIN;
    sda_gpio.Mode  = GPIO_MODE_OUTPUT_OD;
    sda_gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    sda_gpio.Pull  = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &sda_gpio);
    HAL_GPIO_WritePin(GPIOB, PIN_OLED_SDA_PIN, GPIO_PIN_SET);

    /* Configure SCL as open-drain output for bus recovery */
    GPIO_InitTypeDef scl_gpio = {0};
    scl_gpio.Pin   = PIN_OLED_SCL_PIN;
    scl_gpio.Mode  = GPIO_MODE_OUTPUT_OD;
    scl_gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    scl_gpio.Pull  = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &scl_gpio);

    /* Toggle SCL 9+ times to clear any stuck slave */
    for (int i = 0; i < 18; i++) {
        HAL_GPIO_WritePin(GPIOB, PIN_OLED_SCL_PIN, GPIO_PIN_RESET);
        for (volatile int d = 0; d < 200; d++) { __NOP(); }
        HAL_GPIO_WritePin(GPIOB, PIN_OLED_SCL_PIN, GPIO_PIN_SET);
        for (volatile int d = 0; d < 200; d++) { __NOP(); }
    }

    /* Generate STOP condition: SDA low→high while SCL high */
    HAL_GPIO_WritePin(GPIOB, PIN_OLED_SDA_PIN, GPIO_PIN_RESET);
    for (volatile int d = 0; d < 200; d++) { __NOP(); }
    HAL_GPIO_WritePin(GPIOB, PIN_OLED_SCL_PIN, GPIO_PIN_SET);
    for (volatile int d = 0; d < 200; d++) { __NOP(); }
    HAL_GPIO_WritePin(GPIOB, PIN_OLED_SDA_PIN, GPIO_PIN_SET);
    HAL_Delay(1);

    /* Now switch both pins to I2C alternate function */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = PIN_OLED_SDA_PIN | PIN_OLED_SCL_PIN;
    gpio.Mode  = GPIO_MODE_AF_OD;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Pull  = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &gpio);

    /* Reset I2C peripheral again after GPIO switch */
    __HAL_RCC_I2C1_FORCE_RESET();
    HAL_Delay(1);
    __HAL_RCC_I2C1_RELEASE_RESET();

    s_hi2c1.Instance             = I2C1;
    s_hi2c1.Init.ClockSpeed      = 100000;
    s_hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    s_hi2c1.Init.OwnAddress1     = 0;
    s_hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    if (HAL_I2C_Init(&s_hi2c1) != HAL_OK) {
        /* I2C init failed */
        return false;
    }

    HAL_Delay(50);  /* Let bus stabilize */

    /* Check if OLED is present — try 0x3C first, then 0x3D */
    s_oled_i2c_addr = OLED_I2C_ADDR;
    if (HAL_I2C_IsDeviceReady(&s_hi2c1, SSD1306_ADDR, 5, 200) != HAL_OK) {
        s_oled_i2c_addr = 0x3D;
        if (HAL_I2C_IsDeviceReady(&s_hi2c1, SSD1306_ADDR, 5, 200) != HAL_OK) {
            /* OLED not found at 0x3C or 0x3D */
            return false;
        }
    }

    /* Common init commands (same for SSD1306 and SH1106) */
    WriteCmd(SSD1306_DISPLAY_OFF);
    HAL_Delay(10);
    WriteCmd(SSD1306_SET_CLK_DIV);    WriteCmd(0x80);
    WriteCmd(SSD1306_SET_MUX);        WriteCmd(63);
    WriteCmd(SSD1306_SET_OFFSET);     WriteCmd(0);
    WriteCmd(SSD1306_SET_START_LINE | 0);

    /* Send BOTH charge pump variants — the chip ignores the one it
     * doesn't understand. Covers both SSD1306 and SH1106 modules.
     * SSD1306: 0x8D, 0x14    SH1106: 0xAD, 0x8B */
    WriteCmd(0x8D);  WriteCmd(0x14);   /* SSD1306-style */
    WriteCmd(0xAD);  WriteCmd(0x8B);   /* SH1106-style */

    WriteCmd(SSD1306_SEG_REMAP | 1);
    WriteCmd(SSD1306_COM_SCAN_DEC);
    WriteCmd(SSD1306_SET_COM_PINS);   WriteCmd(0x12);
    WriteCmd(SSD1306_SET_CONTRAST);   WriteCmd(0xFF);
    WriteCmd(SSD1306_ENTIRE_ON_RES);
    WriteCmd(SSD1306_NORMAL_DISPLAY);

    HAL_Delay(150);
    WriteCmd(SSD1306_DISPLAY_ON);
    HAL_Delay(150);

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
        case PAGE_NETWORK: RenderNetworkPage(); break;
        case PAGE_WEATHER: RenderWeatherPage(); break;
        case PAGE_VALVES:  RenderValvesPage();  break;
        case PAGE_ALERTS:  RenderAlertsPage();  break;
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
    RenderBig(0, "SASYAMITHRA");
    RenderBig(3, "MOTOR v" FW_VERSION_STR);
    RenderLine(6, "3PH SMART GATEWAY");
}

void Display_Error(const char* line1, const char* line2)
{
    if (!s_initialized) return;
    ClearScreen();
    RenderLine(2, "!!! ERROR !!!");
    if (line1) RenderLine(4, line1);
    if (line2) RenderLine(5, line2);
}
