#include "Chaiss_Communication.h"
#include "chassis_command.h"
#include "vehicle_config.h"
#include "usart.h"
#include "string.h"
#include "math.h"

volatile Chaiss_Communication_t g_chaiss_communication;

static uint8_t uart1_rx_dma[CHAISS_RX_DMA_SIZE];
static uint8_t stream_buffer[CHAISS_FRAME_SIZE * 2u];
static uint16_t stream_length;
static volatile float yaw_rate_kp = YAW_RATE_KP;
static volatile float yaw_rate_ki = YAW_RATE_KI;
static volatile uint8_t yaw_rate_config_version;
static volatile uint8_t yaw_rate_telemetry_enabled;
static volatile uint32_t yaw_rate_tuning_last_ms;
static volatile uint8_t tuning_ack_pending;
static volatile uint8_t tuning_ack_status;
static volatile float tuning_ack_kp;
static volatile float tuning_ack_ki;
static volatile uint8_t tuning_ack_generation;
static uint8_t tuning_ack_tx[CHAISS_TUNING_FRAME_SIZE];
static uint8_t tuning_telemetry_tx[CHAISS_TUNING_FRAME_SIZE];
static uint8_t tuning_telemetry_sequence;
static uint32_t tuning_last_telemetry_ms;

static float read_float_le(const uint8_t *bytes)
{
    uint32_t raw = ((uint32_t)bytes[0]) |
                   ((uint32_t)bytes[1] << 8) |
                   ((uint32_t)bytes[2] << 16) |
                   ((uint32_t)bytes[3] << 24);
    float value;
    memcpy(&value, &raw, sizeof(value));
    return value;
}

static void write_float_le(uint8_t *bytes, float value)
{
    uint32_t raw;
    memcpy(&raw, &value, sizeof(raw));
    bytes[0] = (uint8_t)raw;
    bytes[1] = (uint8_t)(raw >> 8);
    bytes[2] = (uint8_t)(raw >> 16);
    bytes[3] = (uint8_t)(raw >> 24);
}

static uint16_t tuning_crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFu;
    uint16_t i;

    for (i = 0u; i < length; i++)
    {
        uint8_t bit;
        crc ^= data[i];
        for (bit = 0u; bit < 8u; bit++)
        {
            crc = (crc & 1u) ?
                  (uint16_t)((crc >> 1u) ^ 0xA001u) :
                  (uint16_t)(crc >> 1u);
        }
    }

    return crc;
}

static void build_tuning_frame_crc(uint8_t *frame)
{
    const uint16_t crc = tuning_crc16(&frame[1], 10u);
    frame[11] = (uint8_t)crc;
    frame[12] = (uint8_t)(crc >> 8u);
    frame[13] = CHAISS_TUNING_FRAME_TAIL;
}

static void queue_tuning_ack(uint8_t status)
{
    tuning_ack_status = status;
    tuning_ack_kp = yaw_rate_kp;
    tuning_ack_ki = yaw_rate_ki;
    tuning_ack_generation++;
    tuning_ack_pending = 1u;
}

static void parse_tuning_config(const uint8_t *frame)
{
    const uint8_t allowed_flags = CHAISS_TUNING_TELEMETRY_FLAG |
                                  CHAISS_TUNING_GAIN_UPDATE_FLAG;
    uint8_t status = 1u;

    if ((frame[2] & (uint8_t)(~allowed_flags)) == 0u)
    {
        const uint8_t update_gains =
            ((frame[2] & CHAISS_TUNING_GAIN_UPDATE_FLAG) != 0u) ? 1u : 0u;
        float requested_kp = yaw_rate_kp;
        float requested_ki = yaw_rate_ki;

        if (update_gains != 0u)
        {
            requested_kp = read_float_le(&frame[3]);
            requested_ki = read_float_le(&frame[7]);
            if (!isfinite(requested_kp) || !isfinite(requested_ki) ||
                (requested_kp < YAW_RATE_KP_MIN) ||
                (requested_kp > YAW_RATE_KP_MAX) ||
                (requested_ki < YAW_RATE_KI_MIN) ||
                (requested_ki > YAW_RATE_KI_MAX))
            {
                queue_tuning_ack(status);
                return;
            }
        }

        yaw_rate_config_version++;
        if (update_gains != 0u)
        {
            yaw_rate_kp = requested_kp;
            yaw_rate_ki = requested_ki;
        }
        yaw_rate_telemetry_enabled =
            ((frame[2] & CHAISS_TUNING_TELEMETRY_FLAG) != 0u) ? 1u : 0u;
        yaw_rate_tuning_last_ms = HAL_GetTick();
        yaw_rate_config_version++;
        status = 0u;
    }

    queue_tuning_ack(status);
}

static void parse_legacy_command(const uint8_t *frame)
{
    float vx;
    float vy;
    float vz;
    uint8_t vy_is_ff_marker;
    uint8_t vy_is_ee_marker;

    vx = read_float_le(&frame[1]);
    vy = read_float_le(&frame[5]);
    vz = read_float_le(&frame[9]);
    vy_is_ff_marker = (frame[5] == 0xFFu && frame[6] == 0xFFu &&
                       frame[7] == 0xFFu && frame[8] == 0xFFu);
    vy_is_ee_marker = (frame[5] == 0xEEu && frame[6] == 0xEEu &&
                       frame[7] == 0xEEu && frame[8] == 0xEEu);

    if (isfinite(vx) && isfinite(vz))
    {
        g_chaiss_communication.last_frame_ms = HAL_GetTick();
        g_chaiss_communication.valid_frame_count++;
        g_chaiss_communication.online = 1u;

        if (g_chaiss_communication.command_enabled != 0u)
        {
            g_chaiss_communication.vx = vx;
            g_chaiss_communication.vy = vy;
            g_chaiss_communication.vz = vz;
            if (vy_is_ff_marker || vy_is_ee_marker)
            {
                g_chaiss_communication.rail_command = vy_is_ff_marker ?
                    CHAISS_RAIL_RETRACT : CHAISS_RAIL_EXTEND;
                g_chaiss_communication.rail_command_ms = HAL_GetTick();
                g_chaiss_communication.rail_command_valid = 1u;
            }
            g_chaiss_communication.accepted_command_count++;

            if (!vy_is_ff_marker && !vy_is_ee_marker)
            {
                /* Differential drive ignores ordinary VY and uses VX/VZ. */
                chassis_command_publish(vx, vz, 1u, CHASSIS_CMD_ARM);
            }
            else
            {
                chassis_command_publish_heading_lock(CHASSIS_CMD_ARM);
            }
        }
        else
        {
            /* Valid frame received while CH8 is not in serial mode. */
            g_chaiss_communication.blocked_frame_count++;
        }
    }
    else
    {
        g_chaiss_communication.invalid_frame_count++;
    }
}

static void remove_stream_prefix(uint16_t count)
{
    if (count >= stream_length)
    {
        stream_length = 0u;
        return;
    }

    memmove(stream_buffer, &stream_buffer[count], stream_length - count);
    stream_length = (uint16_t)(stream_length - count);
}

static void parse_stream(void)
{
    while (stream_length > 0u)
    {
        if ((stream_buffer[0] != CHAISS_FRAME_HEAD) &&
            (stream_buffer[0] != CHAISS_TUNING_FRAME_HEAD))
        {
            remove_stream_prefix(1u);
            g_chaiss_communication.invalid_frame_count++;
            continue;
        }

        if (stream_length < CHAISS_FRAME_SIZE)
        {
            return;
        }

        if (stream_buffer[0] == CHAISS_TUNING_FRAME_HEAD)
        {
            uint8_t type = stream_buffer[1];
            uint16_t received_crc = (uint16_t)stream_buffer[11] |
                                    ((uint16_t)stream_buffer[12] << 8u);

            if (stream_buffer[CHAISS_TUNING_FRAME_SIZE - 1u] !=
                CHAISS_TUNING_FRAME_TAIL)
            {
                remove_stream_prefix(1u);
                g_chaiss_communication.invalid_frame_count++;
                continue;
            }

            if (received_crc != tuning_crc16(&stream_buffer[1], 10u))
            {
                /* Keep scanning: a valid frame may begin inside this payload. */
                remove_stream_prefix(1u);
                g_chaiss_communication.invalid_frame_count++;
                continue;
            }

            if (type == CHAISS_TUNING_TYPE_CONFIG)
            {
                parse_tuning_config(stream_buffer);
            }
            else if ((type != CHAISS_TUNING_TYPE_TELEMETRY) &&
                     (type != CHAISS_TUNING_TYPE_CONFIG_ACK))
            {
                /* Unknown type is malformed; retry alignment after one byte. */
                remove_stream_prefix(1u);
                g_chaiss_communication.invalid_frame_count++;
                continue;
            }
            else
            {
                /* Known MCU-to-host frame received inbound; discard as a whole. */
            }
        }
        else
        {
            if (stream_buffer[CHAISS_FRAME_SIZE - 1u] != CHAISS_FRAME_TAIL)
            {
                remove_stream_prefix(1u);
                g_chaiss_communication.invalid_frame_count++;
                continue;
            }
            parse_legacy_command(stream_buffer);
        }

        remove_stream_prefix(CHAISS_FRAME_SIZE);
    }
}

static void append_rx_bytes(const uint8_t *data, uint16_t size)
{
    uint16_t index = 0u;

    while (index < size)
    {
        uint16_t free_size = (uint16_t)(sizeof(stream_buffer) - stream_length);
        uint16_t copy_size;

        if (free_size == 0u)
        {
            remove_stream_prefix(1u);
            free_size = 1u;
            g_chaiss_communication.invalid_frame_count++;
        }

        copy_size = (uint16_t)(size - index);
        if (copy_size > free_size)
        {
            copy_size = free_size;
        }

        memcpy(&stream_buffer[stream_length], &data[index], copy_size);
        stream_length = (uint16_t)(stream_length + copy_size);
        index = (uint16_t)(index + copy_size);
        parse_stream();
    }
}

void Chaiss_Communication_Init(void)
{
    memset((void *)&g_chaiss_communication, 0,
           sizeof(g_chaiss_communication));
    stream_length = 0u;
    memset(uart1_rx_dma, 0, sizeof(uart1_rx_dma));
    yaw_rate_kp = YAW_RATE_KP;
    yaw_rate_ki = YAW_RATE_KI;
    yaw_rate_config_version = 0u;
    yaw_rate_telemetry_enabled = 0u;
    yaw_rate_tuning_last_ms = 0u;
    tuning_ack_pending = 0u;
    tuning_ack_status = 0u;
    tuning_ack_kp = YAW_RATE_KP;
    tuning_ack_ki = YAW_RATE_KI;
    tuning_ack_generation = 0u;
    tuning_telemetry_sequence = 0u;
    tuning_last_telemetry_ms = HAL_GetTick();

    (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart1,
                                       uart1_rx_dma,
                                       CHAISS_RX_DMA_SIZE);
    if (huart1.hdmarx != 0)
    {
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    }
}

void Chaiss_Communication_RxEvent(uint16_t size)
{
    if (size > CHAISS_RX_DMA_SIZE)
    {
        size = CHAISS_RX_DMA_SIZE;
    }

    append_rx_bytes(uart1_rx_dma, size);
    (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart1,
                                       uart1_rx_dma,
                                       CHAISS_RX_DMA_SIZE);
    if (huart1.hdmarx != 0)
    {
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    }
}

void Chaiss_Communication_UARTError(void)
{
    (void)HAL_UART_AbortReceive(&huart1);
    (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart1,
                                       uart1_rx_dma,
                                       CHAISS_RX_DMA_SIZE);
    if (huart1.hdmarx != 0)
    {
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    }
}

void Chaiss_Communication_SetCommandEnabled(uint8_t enabled)
{
    g_chaiss_communication.command_enabled = (enabled != 0u) ? 1u : 0u;
    if (enabled == 0u)
    {
        g_chaiss_communication.rail_command = CHAISS_RAIL_STOP;
        g_chaiss_communication.rail_command_valid = 0u;
        g_chaiss_communication.rail_command_ms = 0u;
    }
}

uint8_t Chaiss_Communication_IsOnline(void)
{
    if ((g_chaiss_communication.online != 0u) &&
        ((HAL_GetTick() - g_chaiss_communication.last_frame_ms) <=
         CHAISS_ONLINE_TIMEOUT_MS))
    {
        return 1u;
    }

    g_chaiss_communication.online = 0u;
    return 0u;
}

uint8_t Chaiss_Communication_HasCommand(void)
{
    return (g_chaiss_communication.accepted_command_count > 0u) ? 1u : 0u;
}

void Chaiss_Communication_GetYawRateTuning(
    Chaiss_YawRateTuningConfig_t *config)
{
    uint8_t version_before;
    uint8_t version_after;

    if (config == 0)
    {
        return;
    }

    for (;;)
    {
        version_before = yaw_rate_config_version;
        if ((version_before & 1u) != 0u)
        {
            continue;
        }

        config->kp = yaw_rate_kp;
        config->ki = yaw_rate_ki;
        config->telemetry_enabled = yaw_rate_telemetry_enabled;
        version_after = yaw_rate_config_version;
        if ((version_before == version_after) &&
            ((version_after & 1u) == 0u))
        {
            break;
        }
    }

    if ((config->telemetry_enabled != 0u) &&
        ((HAL_GetTick() - yaw_rate_tuning_last_ms) >
         CHAISS_TUNING_HEARTBEAT_TIMEOUT_MS))
    {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        if ((yaw_rate_telemetry_enabled != 0u) &&
            ((HAL_GetTick() - yaw_rate_tuning_last_ms) >
             CHAISS_TUNING_HEARTBEAT_TIMEOUT_MS))
        {
            yaw_rate_telemetry_enabled = 0u;
        }
        config->telemetry_enabled = yaw_rate_telemetry_enabled;
        if (primask == 0u)
        {
            __enable_irq();
        }
    }
}

void Chaiss_Communication_ProcessTx(uint32_t now_ms,
                                    float target_radps,
                                    float actual_radps)
{
    Chaiss_YawRateTuningConfig_t config;
    HAL_StatusTypeDef tx_status;

    Chaiss_Communication_GetYawRateTuning(&config);

    if (huart1.gState != HAL_UART_STATE_READY)
    {
        return;
    }

    if (tuning_ack_pending != 0u)
    {
        uint8_t ack_generation;
        uint8_t ack_status;
        float ack_kp;
        float ack_ki;

        ack_generation = tuning_ack_generation;
        ack_status = tuning_ack_status;
        ack_kp = tuning_ack_kp;
        ack_ki = tuning_ack_ki;

        tuning_ack_tx[0] = CHAISS_TUNING_FRAME_HEAD;
        tuning_ack_tx[1] = CHAISS_TUNING_TYPE_CONFIG_ACK;
        tuning_ack_tx[2] = ack_status;
        write_float_le(&tuning_ack_tx[3], ack_kp);
        write_float_le(&tuning_ack_tx[7], ack_ki);
        build_tuning_frame_crc(tuning_ack_tx);

        tx_status = HAL_UART_Transmit_DMA(&huart1, tuning_ack_tx,
                                         CHAISS_TUNING_FRAME_SIZE);
        if (tx_status == HAL_OK)
        {
            uint32_t primask = __get_PRIMASK();
            __disable_irq();
            if (tuning_ack_generation == ack_generation)
            {
                tuning_ack_pending = 0u;
            }
            if (primask == 0u)
            {
                __enable_irq();
            }
        }
        return;
    }

    if ((config.telemetry_enabled == 0u) ||
        ((now_ms - tuning_last_telemetry_ms) <
         CHAISS_TUNING_TELEMETRY_PERIOD_MS))
    {
        return;
    }

    tuning_telemetry_tx[0] = CHAISS_TUNING_FRAME_HEAD;
    tuning_telemetry_tx[1] = CHAISS_TUNING_TYPE_TELEMETRY;
    tuning_telemetry_tx[2] = tuning_telemetry_sequence;
    write_float_le(&tuning_telemetry_tx[3], target_radps);
    write_float_le(&tuning_telemetry_tx[7], actual_radps);
    build_tuning_frame_crc(tuning_telemetry_tx);

    tx_status = HAL_UART_Transmit_DMA(&huart1, tuning_telemetry_tx,
                                     CHAISS_TUNING_FRAME_SIZE);
    if (tx_status == HAL_OK)
    {
        tuning_telemetry_sequence++;
        tuning_last_telemetry_ms = now_ms;
    }
}

Chaiss_RailCommand Chaiss_Communication_GetRailCommand(void)
{
    if ((g_chaiss_communication.command_enabled == 0u) ||
        (g_chaiss_communication.rail_command_valid == 0u) ||
        ((HAL_GetTick() - g_chaiss_communication.rail_command_ms) >
         CHAISS_ONLINE_TIMEOUT_MS))
    {
        return CHAISS_RAIL_STOP;
    }

    return (Chaiss_RailCommand)g_chaiss_communication.rail_command;
}
