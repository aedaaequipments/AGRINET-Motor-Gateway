/**
 * @file offline_queue.h
 * @brief Firebase Offline Queue - buffers data when GSM is down
 *
 * Circular buffer of 8 JSON packets. When GSM reconnects, queued
 * packets are sent in order. Oldest packets are dropped if buffer full.
 */

#ifndef OFFLINE_QUEUE_H
#define OFFLINE_QUEUE_H

#include "config.h"
#include "gsm_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

#define OFFLINE_QUEUE_SIZE      8
#define OFFLINE_PKT_MAX_LEN     384

typedef struct {
    HttpMethod_t method;
    char path[128];
    char json[OFFLINE_PKT_MAX_LEN];
    uint32_t timestamp;
} OfflinePacket_t;

/**
 * @brief Initialize offline queue
 */
void OfflineQueue_Init(void);

/**
 * @brief Enqueue a packet for later sending
 * @return true if queued, false if queue full (oldest dropped)
 */
bool OfflineQueue_Enqueue(HttpMethod_t method, const char* path, const char* json);

/**
 * @brief Try to send all queued packets (call when GSM reconnects)
 * @return Number of packets successfully sent
 */
uint8_t OfflineQueue_Flush(void);

/**
 * @brief Get number of packets in queue
 */
uint8_t OfflineQueue_Count(void);

/**
 * @brief Check if queue is empty
 */
bool OfflineQueue_IsEmpty(void);

#ifdef __cplusplus
}
#endif

#endif /* OFFLINE_QUEUE_H */
