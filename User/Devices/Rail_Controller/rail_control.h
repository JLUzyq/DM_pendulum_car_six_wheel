#ifndef __RAIL_CONTROL_H
#define __RAIL_CONTROL_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    RAIL_STOP = 0,
    RAIL_FORWARD,
    RAIL_REVERSE
} RailDirection;

typedef struct
{
    UART_HandleTypeDef *uart;
    GPIO_TypeDef *de_port;       /* 自动收发 RS485 留空；手动 DE 时指定 GPIO */
    uint16_t de_pin;
    uint8_t slave_address;
    RailDirection direction;
    bool direction_valid;        /* false 表示继电器状态未知，需要重新发送 */
} RailController;

void Rail_Init(RailController *rail, UART_HandleTypeDef *uart,
               GPIO_TypeDef *de_port, uint16_t de_pin, uint8_t slave_address);
bool Rail_SetDirection(RailController *rail, RailDirection wanted);
bool Rail_Stop(RailController *rail);

#endif /* __RAIL_CONTROL_H */
