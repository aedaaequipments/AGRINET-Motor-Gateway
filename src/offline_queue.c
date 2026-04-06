/**
 * @file offline_queue.c
 * @brief MQTT Offline Queue - circular buffer for broker outages
 */

#include "offline_queue.h"
#include "gsm_driver.h"
#include <string.h>

static OfflinePacket_t s_queue[OFFLINE_QUEUE_SIZE];
static uint8_t s_head = 0;
static uint8_t s_tail = 0;
static uint8_t s_count = 0;

void OfflineQueue_Init(void)
{
    s_head = 0;
    s_tail = 0;
    s_count = 0;
    memset(s_queue, 0, sizeof(s_queue));
}

bool OfflineQueue_Enqueue(const char* topic, const char* json, uint8_t qos)
{
    if (s_count >= OFFLINE_QUEUE_SIZE) {
        s_tail = (s_tail + 1) % OFFLINE_QUEUE_SIZE;
        s_count--;
    }

    OfflinePacket_t* pkt = &s_queue[s_head];
    strncpy(pkt->topic, topic, OFFLINE_TOPIC_MAX_LEN - 1);
    pkt->topic[OFFLINE_TOPIC_MAX_LEN - 1] = '\0';

    if (json) {
        strncpy(pkt->json, json, OFFLINE_PKT_MAX_LEN - 1);
        pkt->json[OFFLINE_PKT_MAX_LEN - 1] = '\0';
    } else {
        pkt->json[0] = '\0';
    }

    pkt->qos = qos;
    pkt->timestamp = HAL_GetTick();

    s_head = (s_head + 1) % OFFLINE_QUEUE_SIZE;
    s_count++;
    return true;
}

uint8_t OfflineQueue_Flush(void)
{
    if (!GSM_IsReady() || s_count == 0) return 0;

    uint8_t sent = 0;

    while (s_count > 0) {
        OfflinePacket_t* pkt = &s_queue[s_tail];

        if (GSM_MqttPublish(pkt->topic, pkt->json, pkt->qos)) {
            sent++;
            s_tail = (s_tail + 1) % OFFLINE_QUEUE_SIZE;
            s_count--;
        } else {
            break;
        }
    }

    return sent;
}

uint8_t OfflineQueue_Count(void) { return s_count; }
bool OfflineQueue_IsEmpty(void)  { return s_count == 0; }
