/**
 * @file node_registry.h
 * @brief AGRINET Field Node Registry - tracks weather stations and valves
 *
 * Static array of up to 8 nodes. Tracks:
 * - Device UID and type
 * - Last seen timestamp
 * - Online/offline status
 * - Last received data
 */

#ifndef NODE_REGISTRY_H
#define NODE_REGISTRY_H

#include "config.h"
#include "agrinet_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_NODES           8
#define NODE_TIMEOUT_MS     120000  // 2 minutes = offline

typedef struct {
    uint64_t uid;                   // Device UID
    uint8_t  type;                  // NODE_TYPE_WEATHER, NODE_TYPE_VALVE, etc.
    uint32_t lastSeenTick;          // HAL_GetTick() of last packet
    int16_t  rssi;                  // Last RSSI
    bool     online;                // Currently online
    bool     registered;            // Slot in use

    /* Cached data */
    union {
        WeatherPayload_t weather;
        ValvePayload_t   valve;
    } data;
    bool dataValid;                 // Has received at least one payload
} NodeEntry_t;

/**
 * @brief Initialize node registry
 */
void NodeRegistry_Init(void);

/**
 * @brief Register or update a node (called when packet received)
 * @param uid Device UID
 * @param type Device type
 * @param rssi Signal strength
 * @return Pointer to node entry, or NULL if registry full
 */
NodeEntry_t* NodeRegistry_Update(uint64_t uid, uint8_t type, int16_t rssi);

/**
 * @brief Find a node by UID
 * @return Node entry or NULL
 */
NodeEntry_t* NodeRegistry_Find(uint64_t uid);

/**
 * @brief Find first node of given type
 * @param type NODE_TYPE_WEATHER, NODE_TYPE_VALVE, etc.
 * @return Node entry or NULL
 */
NodeEntry_t* NodeRegistry_FindByType(uint8_t type);

/**
 * @brief Get count of online nodes
 */
uint8_t NodeRegistry_OnlineCount(void);

/**
 * @brief Check timeouts and mark offline nodes (call periodically)
 */
void NodeRegistry_CheckTimeouts(void);

/**
 * @brief Get node array pointer (for iteration)
 * @param count Output: number of registered nodes
 * @return Array of NodeEntry_t
 */
const NodeEntry_t* NodeRegistry_GetAll(uint8_t* count);

#ifdef __cplusplus
}
#endif

#endif /* NODE_REGISTRY_H */
