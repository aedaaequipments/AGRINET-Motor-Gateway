/**
 * @file node_registry.c
 * @brief AGRINET Field Node Registry
 */

#include "node_registry.h"
#include <string.h>

static NodeEntry_t g_nodes[MAX_NODES];

void NodeRegistry_Init(void)
{
    memset(g_nodes, 0, sizeof(g_nodes));
}

NodeEntry_t* NodeRegistry_Update(uint64_t uid, uint8_t type, int16_t rssi)
{
    /* Find existing */
    for (uint8_t i = 0; i < MAX_NODES; i++) {
        if (g_nodes[i].registered && g_nodes[i].uid == uid) {
            g_nodes[i].lastSeenTick = HAL_GetTick();
            g_nodes[i].rssi = rssi;
            g_nodes[i].online = true;
            return &g_nodes[i];
        }
    }

    /* Find empty slot */
    for (uint8_t i = 0; i < MAX_NODES; i++) {
        if (!g_nodes[i].registered) {
            g_nodes[i].uid = uid;
            g_nodes[i].type = type;
            g_nodes[i].lastSeenTick = HAL_GetTick();
            g_nodes[i].rssi = rssi;
            g_nodes[i].online = true;
            g_nodes[i].registered = true;
            g_nodes[i].dataValid = false;
            return &g_nodes[i];
        }
    }

    return NULL;  // Registry full
}

NodeEntry_t* NodeRegistry_Find(uint64_t uid)
{
    for (uint8_t i = 0; i < MAX_NODES; i++) {
        if (g_nodes[i].registered && g_nodes[i].uid == uid) {
            return &g_nodes[i];
        }
    }
    return NULL;
}

NodeEntry_t* NodeRegistry_FindByType(uint8_t type)
{
    for (uint8_t i = 0; i < MAX_NODES; i++) {
        if (g_nodes[i].registered && g_nodes[i].type == type) {
            return &g_nodes[i];
        }
    }
    return NULL;
}

uint8_t NodeRegistry_OnlineCount(void)
{
    uint8_t count = 0;
    for (uint8_t i = 0; i < MAX_NODES; i++) {
        if (g_nodes[i].registered && g_nodes[i].online) count++;
    }
    return count;
}

void NodeRegistry_CheckTimeouts(void)
{
    uint32_t now = HAL_GetTick();
    for (uint8_t i = 0; i < MAX_NODES; i++) {
        if (g_nodes[i].registered && g_nodes[i].online) {
            if ((now - g_nodes[i].lastSeenTick) > NODE_TIMEOUT_MS) {
                g_nodes[i].online = false;
            }
        }
    }
}

const NodeEntry_t* NodeRegistry_GetAll(uint8_t* count)
{
    *count = 0;
    for (uint8_t i = 0; i < MAX_NODES; i++) {
        if (g_nodes[i].registered) (*count)++;
    }
    return g_nodes;
}
