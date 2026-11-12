#include "rail_control.h"

#include "cmsis_os.h"

#define MODBUS_TX_TIMEOUT_MS     30u
#define MODBUS_REPLY_TIMEOUT_MS  50u
#define RELAY_RELEASE_MS         80u

static uint16_t modbus_crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFu;
    uint16_t i;

    for (i = 0u; i < length; i++)
    {
        uint8_t bit;
        crc ^= data[i];
        for (bit = 0u; bit < 8u; bit++)
        {
            crc = (crc & 1u) ? (uint16_t)((crc >> 1u) ^ 0xA001u)
                             : (uint16_t)(crc >> 1u);
        }
    }
    return crc;
}

static bool write_coil(RailController *rail, uint16_t coil, bool on)
{
    const uint16_t value = on ? 0xFF00u : 0x0000u;
    uint8_t frame[8] = {
        rail->slave_address, 0x05u,
        (uint8_t)(coil >> 8u), (uint8_t)coil,
        (uint8_t)(value >> 8u), (uint8_t)value, 0u, 0u
    };
    uint8_t reply[8];
    uint8_t i;
    const uint16_t crc = modbus_crc16(frame, 6u);

    frame[6] = (uint8_t)crc;
    frame[7] = (uint8_t)(crc >> 8u);

    if (rail->de_port != NULL)
    {
        HAL_GPIO_WritePin(rail->de_port, rail->de_pin, GPIO_PIN_SET);
    }
    if (HAL_UART_Transmit(rail->uart, frame, 8u, MODBUS_TX_TIMEOUT_MS) != HAL_OK)
    {
        if (rail->de_port != NULL)
        {
            HAL_GPIO_WritePin(rail->de_port, rail->de_pin, GPIO_PIN_RESET);
        }
        return false;
    }
    if (rail->de_port != NULL)
    {
        HAL_GPIO_WritePin(rail->de_port, rail->de_pin, GPIO_PIN_RESET);
    }

    if (HAL_UART_Receive(rail->uart, reply, 8u, MODBUS_REPLY_TIMEOUT_MS) != HAL_OK)
    {
        return false;
    }
    for (i = 0u; i < 8u; i++)
    {
        if (reply[i] != frame[i])
        {
            return false;
        }
    }
    return true;
}

static bool all_off(RailController *rail)
{
    const bool k1_ok = write_coil(rail, 0u, false);
    const bool k2_ok = write_coil(rail, 1u, false);
    if (k1_ok && k2_ok)
    {
        rail->direction = RAIL_STOP;
        rail->direction_valid = true;
        return true;
    }
    rail->direction_valid = false;
    return false;
}

void Rail_Init(RailController *rail, UART_HandleTypeDef *uart,
               GPIO_TypeDef *de_port, uint16_t de_pin, uint8_t slave_address)
{
    rail->uart = uart;
    rail->de_port = de_port;
    rail->de_pin = de_pin;
    rail->slave_address = slave_address;
    rail->direction = RAIL_STOP;
    rail->direction_valid = false;
    if (de_port != NULL)
    {
        HAL_GPIO_WritePin(de_port, de_pin, GPIO_PIN_RESET);
    }
    (void)all_off(rail);
}

bool Rail_Stop(RailController *rail)
{
    if (rail->direction_valid && rail->direction == RAIL_STOP)
    {
        return true;
    }
    return all_off(rail);
}

bool Rail_SetDirection(RailController *rail, RailDirection wanted)
{
    uint16_t coil;

    if (wanted == RAIL_STOP)
    {
        return Rail_Stop(rail);
    }
    if (wanted != RAIL_FORWARD && wanted != RAIL_REVERSE)
    {
        return false;
    }
    if (rail->direction_valid && wanted == rail->direction)
    {
        return true;
    }

    /* 先确认两路继电器均已释放，避免换向时短接电源。 */
    if (!all_off(rail))
    {
        return false;
    }
    osDelay(RELAY_RELEASE_MS);

    coil = (wanted == RAIL_FORWARD) ? 0u : 1u;
    if (!write_coil(rail, coil, true))
    {
        (void)all_off(rail);
        return false;
    }
    rail->direction = wanted;
    rail->direction_valid = true;
    return true;
}
