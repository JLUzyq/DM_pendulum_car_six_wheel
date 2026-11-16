#ifndef __CHAISS_COMMUNICATION_H
#define __CHAISS_COMMUNICATION_H

#include "main.h"

/*
 * Upper-computer chassis command frame (14 bytes):
 *   byte 0      : 0xA5
 *   bytes 1..4  : VX, float32, little-endian, m/s
 *   bytes 5..8  : VY, float32, little-endian; ignored except raw FF/EE
 *                 patterns, which command the rail
 *   bytes 9..12 : VZ, float32, little-endian, rad/s
 *   byte 13     : 0x4A
 */
#define CHAISS_FRAME_HEAD       (0xA5u)
#define CHAISS_FRAME_TAIL       (0x4Au)
#define CHAISS_FRAME_SIZE       (14u)
#define CHAISS_RX_DMA_SIZE      (64u)
#define CHAISS_ONLINE_TIMEOUT_MS (200u)

/* Optional yaw-rate tuning frames share USART1 with the legacy command stream. */
#define CHAISS_TUNING_FRAME_HEAD              (0xC3u)
#define CHAISS_TUNING_FRAME_TAIL              (0x3Cu)
#define CHAISS_TUNING_FRAME_SIZE              (14u)
#define CHAISS_TUNING_TYPE_CONFIG             (0x01u)
#define CHAISS_TUNING_TYPE_TELEMETRY          (0x81u)
#define CHAISS_TUNING_TYPE_CONFIG_ACK         (0x82u)
#define CHAISS_TUNING_TYPE_GAIN_QUERY         (0x83u)
#define CHAISS_TUNING_TELEMETRY_FLAG          (0x01u)
#define CHAISS_TUNING_GAIN_UPDATE_FLAG        (0x02u)
#define CHAISS_TUNING_QUERY_GAINS_FLAG        (0x04u)
#define CHAISS_TUNING_HEARTBEAT_TIMEOUT_MS    (1500u)
#define CHAISS_TUNING_TELEMETRY_PERIOD_MS     (50u)

typedef enum
{
    CHAISS_RAIL_STOP = 0,
    CHAISS_RAIL_EXTEND,
    CHAISS_RAIL_RETRACT
} Chaiss_RailCommand;

typedef struct
{
    float kp;
    float ki;
    uint8_t telemetry_enabled;
} Chaiss_YawRateTuningConfig_t;

typedef struct
{
    float vx;
    float vy;
    float vz;
    uint32_t last_frame_ms;
    uint32_t valid_frame_count;
    uint32_t accepted_command_count;
    uint32_t blocked_frame_count;
    uint32_t invalid_frame_count;
    uint8_t online;
    uint8_t command_enabled;
    uint8_t rail_command;
    uint8_t rail_command_valid;
    uint32_t rail_command_ms;
} Chaiss_Communication_t;

extern volatile Chaiss_Communication_t g_chaiss_communication;

void Chaiss_Communication_Init(void);
void Chaiss_Communication_RxEvent(uint16_t size);
void Chaiss_Communication_UARTError(void);
void Chaiss_Communication_SetCommandEnabled(uint8_t enabled);
uint8_t Chaiss_Communication_IsOnline(void);
Chaiss_RailCommand Chaiss_Communication_GetRailCommand(void);
/* Returns 1 after the first accepted serial command, until the MCU is reset. */
uint8_t Chaiss_Communication_HasCommand(void);
void Chaiss_Communication_GetYawRateTuning(
    Chaiss_YawRateTuningConfig_t *config);
/* Call from the chassis task; pending ACKs have priority over telemetry. */
void Chaiss_Communication_ProcessTx(uint32_t now_ms,
                                    float target_radps,
                                    float actual_radps);

#endif /* __CHAISS_COMMUNICATION_H */
