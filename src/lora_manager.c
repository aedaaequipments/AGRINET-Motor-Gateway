/**
 * @file lora_manager.c
 * @brief LoRa Network Manager - master gateway coordination
 *
 * Handles:
 * - Receiving weather data from AGN-W nodes
 * - Receiving valve ACKs from AGN-V nodes
 * - Sending valve commands
 * - Sending ACKs for received data
 * - IN865 channel hopping with duty cycle tracking
 */

#include "lora_manager.h"
#include "lora_radio.h"
#include "node_registry.h"
#include "cloud_sync.h"
#include "flash_config.h"
#include <string.h>

/* ═══════════════════════════════════════════════════════════════════════════
 * PRIVATE DATA
 * ═══════════════════════════════════════════════════════════════════════════ */

static uint64_t s_masterUid;
static uint32_t s_msgIdCounter = 0;

/* Packet buffers */
static uint8_t s_txBuf[LORA_MAX_PACKET_SIZE];
static uint8_t s_rxBuf[LORA_MAX_PACKET_SIZE];

/* Latest weather data */
static WeatherPayload_t s_lastWeather;
static uint64_t s_lastWeatherUid = 0;
static bool s_newWeatherData = false;

/* Channel hopping */
static const uint32_t s_channels[IN865_CHANNEL_COUNT] = {
    IN865_CHANNEL_0, IN865_CHANNEL_1, IN865_CHANNEL_2
};
static uint8_t s_currentChannel = 0;
static uint32_t s_channelTxTime[IN865_CHANNEL_COUNT] = {0};  // TX time in ms per channel
static uint32_t s_channelResetTick = 0;  // Reset duty cycle tracking every hour

/* ═══════════════════════════════════════════════════════════════════════════
 * HELPERS
 * ═══════════════════════════════════════════════════════════════════════════ */

static uint32_t NextMsgId(void) { return ++s_msgIdCounter; }

static void HopChannel(void)
{
    s_currentChannel = (s_currentChannel + 1) % IN865_CHANNEL_COUNT;
    LoRa_SetFrequency(s_channels[s_currentChannel]);
}

/**
 * Build and send an ACK packet
 */
static void SendAck(uint64_t dstUid, uint32_t msgId)
{
    LoRaHeader_t hdr;
    hdr.sync   = LORA_SYNC_WORD_PACKET;
    hdr.length = sizeof(LoRaHeader_t) + LORA_CRC_SIZE;
    hdr.flags  = FLAG_VERSION_1 |
                 ((FRAME_TYPE_ACK << FLAG_TYPE_SHIFT) & FLAG_TYPE_MASK);
    hdr.srcUid = s_masterUid;
    hdr.dstUid = dstUid;
    hdr.msgId  = msgId;

    memcpy(s_txBuf, &hdr, sizeof(hdr));
    uint16_t crc = agrinet_crc16(s_txBuf, sizeof(hdr));
    s_txBuf[sizeof(hdr)]     = crc & 0xFF;
    s_txBuf[sizeof(hdr) + 1] = (crc >> 8) & 0xFF;

    LoRa_Send(s_txBuf, sizeof(hdr) + 2);
}

/**
 * Build a command packet
 */
static size_t BuildCommandPacket(uint64_t dstUid, const CommandPayload_t* cmd)
{
    LoRaHeader_t hdr;
    hdr.sync   = LORA_SYNC_WORD_PACKET;
    hdr.flags  = FLAG_VERSION_1 |
                 ((FRAME_TYPE_CMD << FLAG_TYPE_SHIFT) & FLAG_TYPE_MASK) |
                 FLAG_ACK_REQUIRED;
    hdr.srcUid = s_masterUid;
    hdr.dstUid = dstUid;
    hdr.msgId  = NextMsgId();

    size_t totalLen = sizeof(LoRaHeader_t) + sizeof(CommandPayload_t) + LORA_CRC_SIZE;
    hdr.length = (uint8_t)totalLen;

    size_t offset = 0;
    memcpy(s_txBuf + offset, &hdr, sizeof(hdr));
    offset += sizeof(hdr);
    memcpy(s_txBuf + offset, cmd, sizeof(CommandPayload_t));
    offset += sizeof(CommandPayload_t);

    uint16_t crc = agrinet_crc16(s_txBuf, offset);
    s_txBuf[offset++] = crc & 0xFF;
    s_txBuf[offset++] = (crc >> 8) & 0xFF;

    return offset;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * PACKET HANDLER
 * ═══════════════════════════════════════════════════════════════════════════ */

static void HandleReceivedPacket(const uint8_t* buf, uint8_t len)
{
    if (len < sizeof(LoRaHeader_t) + LORA_CRC_SIZE) return;

    /* Parse header */
    LoRaHeader_t hdr;
    memcpy(&hdr, buf, sizeof(hdr));

    /* Validate sync */
    if (hdr.sync != LORA_SYNC_WORD_PACKET) return;

    /* Validate CRC */
    size_t dataLen = len - LORA_CRC_SIZE;
    uint16_t rxCrc = buf[dataLen] | ((uint16_t)buf[dataLen + 1] << 8);
    uint16_t calcCrc = agrinet_crc16(buf, dataLen);
    if (rxCrc != calcCrc) return;

    /* Check destination (must be for us or broadcast) */
    if (hdr.dstUid != s_masterUid && hdr.dstUid != UID_BROADCAST) return;

    /* Extract payload */
    size_t payloadLen = dataLen - sizeof(LoRaHeader_t);
    const uint8_t* payload = buf + sizeof(LoRaHeader_t);

    /* Get frame type */
    uint8_t frameType = AGRINET_GET_FRAME_TYPE(hdr.flags);

    /* Determine source node type from UID */
    uint8_t nodeType = (uint8_t)(hdr.srcUid >> 32);
    int16_t rssi = LoRa_GetRSSI();

    /* Register/update node */
    NodeEntry_t* node = NodeRegistry_Update(hdr.srcUid, nodeType, rssi);

    switch (frameType) {

    case FRAME_TYPE_DATA:
        /* Weather station data */
        if (nodeType == NODE_TYPE_WEATHER && payloadLen >= sizeof(WeatherPayload_t)) {
            memcpy(&s_lastWeather, payload, sizeof(WeatherPayload_t));
            s_lastWeatherUid = hdr.srcUid;
            s_newWeatherData = true;

            if (node) {
                memcpy(&node->data.weather, payload, sizeof(WeatherPayload_t));
                node->dataValid = true;
            }

            /* Send ACK */
            if (hdr.flags & FLAG_ACK_REQUIRED) {
                SendAck(hdr.srcUid, hdr.msgId);
            }
        }
        /* Valve status data */
        else if (nodeType == NODE_TYPE_VALVE && payloadLen >= sizeof(ValvePayload_t)) {
            if (node) {
                memcpy(&node->data.valve, payload, sizeof(ValvePayload_t));
                node->dataValid = true;
            }

            if (hdr.flags & FLAG_ACK_REQUIRED) {
                SendAck(hdr.srcUid, hdr.msgId);
            }
        }
        break;

    case FRAME_TYPE_ACK:
        /* ACK received -- mark pending command as confirmed */
        break;

    case FRAME_TYPE_HELLO:
        /* Discovery beacon -- just register the node (already done above) */
        SendAck(hdr.srcUid, hdr.msgId);
        break;

    default:
        break;
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * PUBLIC API
 * ═══════════════════════════════════════════════════════════════════════════ */

bool LoRaManager_Init(void)
{
    /* Build master UID from device serial */
    /* Use STM32 unique ID as serial number */
    uint32_t serial = *(uint32_t*)0x1FFFF7E8;  // STM32 unique ID register
    s_masterUid = AGRINET_MAKE_UID(NODE_TYPE_MASTER, serial);

    NodeRegistry_Init();

    if (!LoRa_Init()) return false;

    /* Set initial channel */
    LoRa_SetFrequency(s_channels[0]);
    s_channelResetTick = HAL_GetTick();

    /* Start receiving */
    LoRa_StartReceive();

    return true;
}

void LoRaManager_Update(void)
{
    /* Check for received packets */
    uint8_t len = LoRa_Receive(s_rxBuf, sizeof(s_rxBuf));
    if (len > 0) {
        HandleReceivedPacket(s_rxBuf, len);
    }

    /* Check node timeouts */
    NodeRegistry_CheckTimeouts();

    /* Reset duty cycle tracking every hour */
    if ((HAL_GetTick() - s_channelResetTick) > 3600000) {
        memset(s_channelTxTime, 0, sizeof(s_channelTxTime));
        s_channelResetTick = HAL_GetTick();
    }

    /* Push weather data to cloud if new data available */
    if (s_newWeatherData) {
        s_newWeatherData = false;
        /* Station ID derived from UID -- for now use a default */
        CloudSync_PushWeatherData(&s_lastWeather, "WS-001-MAIN");
    }
}

bool LoRaManager_SendValveCommand(uint64_t dstUid, uint8_t position)
{
    CommandPayload_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.cmdType = CMD_TYPE_VALVE_SET;
    cmd.target  = 0;
    cmd.value   = position;
    cmd.duration = 0;  // Indefinite

    size_t len = BuildCommandPacket(dstUid, &cmd);

    HopChannel();  // Hop before TX
    return LoRa_Send(s_txBuf, len);
}

bool LoRaManager_PollWeatherStation(uint64_t dstUid)
{
    /* Send a HELLO frame to trigger the station to respond with data */
    LoRaHeader_t hdr;
    hdr.sync   = LORA_SYNC_WORD_PACKET;
    hdr.length = sizeof(LoRaHeader_t) + LORA_CRC_SIZE;
    hdr.flags  = FLAG_VERSION_1 |
                 ((FRAME_TYPE_HELLO << FLAG_TYPE_SHIFT) & FLAG_TYPE_MASK);
    hdr.srcUid = s_masterUid;
    hdr.dstUid = dstUid;
    hdr.msgId  = NextMsgId();

    memcpy(s_txBuf, &hdr, sizeof(hdr));
    uint16_t crc = agrinet_crc16(s_txBuf, sizeof(hdr));
    s_txBuf[sizeof(hdr)]     = crc & 0xFF;
    s_txBuf[sizeof(hdr) + 1] = (crc >> 8) & 0xFF;

    HopChannel();
    return LoRa_Send(s_txBuf, sizeof(hdr) + 2);
}

bool LoRaManager_GetLastWeatherData(WeatherPayload_t* data, uint64_t* stationUid)
{
    if (s_lastWeatherUid == 0) return false;
    memcpy(data, &s_lastWeather, sizeof(WeatherPayload_t));
    *stationUid = s_lastWeatherUid;
    return true;
}

bool LoRaManager_HasNewWeatherData(void) { return s_newWeatherData; }
uint64_t LoRaManager_GetMasterUid(void) { return s_masterUid; }
