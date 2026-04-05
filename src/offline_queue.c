/**
 * @file offline_queue.c
 * @brief Firebase Offline Queue - circular buffer for GSM outages
 */

#include "offline_queue.h"
#include "gsm_driver.h"
#include <string.h>

static OfflinePacket_t s_queue[OFFLINE_QUEUE_SIZE];
static uint8_t s_head = 0;  // Next write position
static uint8_t s_tail = 0;  // Next read position
static uint8_t s_count = 0;

void OfflineQueue_Init(void)
{
    s_head = 0;
    s_tail = 0;
    s_count = 0;
    memset(s_queue, 0, sizeof(s_queue));
}

bool OfflineQueue_Enqueue(HttpMethod_t method, const char* path, const char* json)
{
    /* If full, drop oldest (advance tail) */
    if (s_count >= OFFLINE_QUEUE_SIZE) {
        s_tail = (s_tail + 1) % OFFLINE_QUEUE_SIZE;
        s_count--;
    }

    OfflinePacket_t* pkt = &s_queue[s_head];
    pkt->method = method;
    strncpy(pkt->path, path, sizeof(pkt->path) - 1);
    pkt->path[sizeof(pkt->path) - 1] = '\0';

    if (json) {
        strncpy(pkt->json, json, OFFLINE_PKT_MAX_LEN - 1);
        pkt->json[OFFLINE_PKT_MAX_LEN - 1] = '\0';
    } else {
        pkt->json[0] = '\0';
    }

    pkt->timestamp = HAL_GetTick();

    s_head = (s_head + 1) % OFFLINE_QUEUE_SIZE;
    s_count++;
    return true;
}

uint8_t OfflineQueue_Flush(void)
{
    if (!GSM_IsReady() || s_count == 0) return 0;

    uint8_t sent = 0;
    char resp[64];

    while (s_count > 0) {
        OfflinePacket_t* pkt = &s_queue[s_tail];

        const char* body = (pkt->json[0] != '\0') ? pkt->json : NULL;
        uint16_t status = GSM_HttpRequest(pkt->method, pkt->path, body, resp, sizeof(resp));

        if (status >= 200 && status < 300) {
            sent++;
            s_tail = (s_tail + 1) % OFFLINE_QUEUE_SIZE;
            s_count--;
        } else {
            break;  // GSM failed again, stop trying
        }
    }

    return sent;
}

uint8_t OfflineQueue_Count(void) { return s_count; }
bool OfflineQueue_IsEmpty(void)  { return s_count == 0; }
