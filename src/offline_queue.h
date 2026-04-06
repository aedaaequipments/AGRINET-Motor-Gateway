/**
 * @file offline_queue.h
 * @brief MQTT Offline Queue - buffers data when broker connection is down
 *
 * Circular buffer of 8 MQTT packets. When broker reconnects, queued
 * packets are published in order. Oldest packets are dropped if buffer full.
 */

#ifndef OFFLINE_QUEUE_H
#define OFFLINE_QUEUE_H

#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

#define OFFLINE_QUEUE_SIZE      4       // Reduced from 8 to fit 20KB RAM
#define OFFLINE_PKT_MAX_LEN     256     // Reduced from 384 — MQTT payloads < 256
#define OFFLINE_TOPIC_MAX_LEN   64      // Reduced from 128 — topics < 50 chars

typedef struct {
    char topic[OFFLINE_TOPIC_MAX_LEN];
    char json[OFFLINE_PKT_MAX_LEN];
    uint8_t qos;
    uint32_t timestamp;
} OfflinePacket_t;

void OfflineQueue_Init(void);

/**
 * @brief Enqueue an MQTT message for later publishing
 */
bool OfflineQueue_Enqueue(const char* topic, const char* json, uint8_t qos);

/**
 * @brief Publish all queued packets (call when MQTT reconnects)
 * @return Number of packets successfully sent
 */
uint8_t OfflineQueue_Flush(void);

uint8_t OfflineQueue_Count(void);
bool OfflineQueue_IsEmpty(void);

#ifdef __cplusplus
}
#endif

#endif /* OFFLINE_QUEUE_H */
