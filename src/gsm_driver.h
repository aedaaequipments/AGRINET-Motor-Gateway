/**
 * @file gsm_driver.h
 * @brief SIM7670C 4G GSM AT Command Driver (MQTT + HTTP)
 *
 * Provides MQTT publish/subscribe for Sasyamithra self-hosted server.
 * C1/C2 fix: Replaces Firebase HTTP polling with MQTT push/subscribe.
 * Thread-safe via FreeRTOS mutex. Implements exponential backoff on errors.
 */

#ifndef GSM_DRIVER_H
#define GSM_DRIVER_H

#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    GSM_STATE_INIT,
    GSM_STATE_NETWORK_READY,    // Network registered, MQTT not yet started
    GSM_STATE_MQTT_CONNECTING,  // MQTT service started, connecting to broker
    GSM_STATE_MQTT_CONNECTED,   // Connected to MQTT broker, ready for pub/sub
    GSM_STATE_ERROR,
    GSM_STATE_RESETTING,
} GsmState_t;

/**
 * @brief Callback for incoming MQTT messages (from subscribed topics)
 * @param topic  Topic string (null-terminated)
 * @param payload  Payload string (null-terminated, usually JSON)
 * @param payloadLen  Length of payload
 */
typedef void (*MqttMessageCallback)(const char* topic, const char* payload, uint16_t payloadLen);

/**
 * @brief Initialize GSM module (USART3, GPIO, network registration)
 * @return true if module responds and network registered
 */
bool GSM_Init(void);

/**
 * @brief Get current GSM state
 */
GsmState_t GSM_GetState(void);

/**
 * @brief Check if MQTT is connected and ready for operations
 */
bool GSM_IsReady(void);

/**
 * @brief Get network time from GSM (AT+CCLK?)
 */
bool GSM_GetNetworkTime(char* buf, uint16_t bufLen);

/**
 * @brief Get signal quality (RSSI 0-31, 99=unknown)
 */
uint8_t GSM_GetSignalQuality(void);

/* ─── MQTT Operations ──────────────────────────────────────────── */

/**
 * @brief Start MQTT service and connect to broker
 * @param brokerIp  Broker IP address (e.g., "192.168.1.100")
 * @param port      Broker port (e.g., 1883)
 * @param clientId  MQTT client ID (e.g., "MOT-001-A1")
 * @param username  MQTT username (empty string for no auth)
 * @param password  MQTT password (empty string for no auth)
 * @return true if connected successfully
 */
bool GSM_MqttConnect(const char* brokerIp, uint16_t port,
                     const char* clientId,
                     const char* username, const char* password);

/**
 * @brief Publish message to MQTT topic
 * @param topic   Topic string (e.g., "data/farm01/MOT-001-A1/telemetry")
 * @param payload JSON payload string
 * @param qos     QoS level (0 or 1)
 * @return true if published successfully
 */
bool GSM_MqttPublish(const char* topic, const char* payload, uint8_t qos);

/**
 * @brief Subscribe to MQTT topic
 * @param topic  Topic string (supports + and # wildcards)
 * @param qos    QoS level (0 or 1)
 * @return true if subscribed successfully
 */
bool GSM_MqttSubscribe(const char* topic, uint8_t qos);

/**
 * @brief Set callback for incoming MQTT messages
 */
void GSM_MqttSetCallback(MqttMessageCallback cb);

/**
 * @brief Check for and process incoming MQTT messages (URCs)
 * Call this periodically from the cloud sync task.
 */
void GSM_MqttProcessIncoming(void);

/**
 * @brief Disconnect from MQTT broker and stop service
 */
void GSM_MqttDisconnect(void);

/**
 * @brief Reset GSM module (called on persistent errors)
 */
void GSM_Reset(void);

/**
 * @brief Run GSM state machine (call periodically for error recovery)
 */
void GSM_Update(void);

#ifdef __cplusplus
}
#endif

#endif /* GSM_DRIVER_H */
