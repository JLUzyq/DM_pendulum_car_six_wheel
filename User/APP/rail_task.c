#include "rail_task.h"

#include "cmsis_os.h"
#include "rc_sbus.h"
#include "rc_control_mode.h"
#include "rail_control.h"
#include "usart.h"
#include "Chaiss_Communication.h"

#define RAIL_SLAVE_ADDRESS          (1u)
#define RAIL_SWITCH_LOW_THRESHOLD   (1300u)
#define RAIL_SWITCH_HIGH_THRESHOLD  (1700u)
#define RAIL_TASK_PERIOD_MS         (20u)

static RailController rail_controller;

static RailDirection rail_direction_from_sbus(void)
{
    uint8_t ch = g_sbus_ch_map.rail_direction;
    uint16_t raw;
    uint16_t pwm;

    if (rc_sbus.online == 0u || ch >= 10u)
    {
        return RAIL_STOP;
    }

    raw = rc_sbus.ch[ch];
    if (raw < SBUS_CH_MIN || raw > SBUS_CH_MAX)
    {
        return RAIL_STOP;
    }

    pwm = rc_sbus.ch_pwm[ch];
    if (pwm > RAIL_SWITCH_HIGH_THRESHOLD)
    {
        return RAIL_FORWARD;
    }
    if (pwm < RAIL_SWITCH_LOW_THRESHOLD)
    {
        return RAIL_REVERSE;
    }
    return RAIL_STOP;
}

static RailDirection rail_direction_from_serial(void)
{
    switch (Chaiss_Communication_GetRailCommand())
    {
        case CHAISS_RAIL_EXTEND:
            return RAIL_FORWARD;
        case CHAISS_RAIL_RETRACT:
            return RAIL_REVERSE;
        case CHAISS_RAIL_STOP:
        default:
            return RAIL_STOP;
    }
}

void RailControl_Task(void const *argument)
{
    (void)argument;
    Rail_Init(&rail_controller, &huart2, GPIOD, GPIO_PIN_4, RAIL_SLAVE_ADDRESS);

    for (;;)
    {
        rc_control_mode_e control_mode = RC_CONTROL_DISABLED;
        uint8_t mode_channel = g_sbus_ch_map.control_mode;
        RailDirection wanted = RAIL_STOP;

        if ((rc_sbus.online != 0u) && (mode_channel < 10u))
        {
            control_mode = rc_control_mode_from_pwm(
                rc_sbus.ch_pwm[mode_channel]);
        }

        if (control_mode == RC_CONTROL_SERIAL)
        {
            wanted = rail_direction_from_serial();
        }
        else if (control_mode == RC_CONTROL_SBUS)
        {
            wanted = rail_direction_from_sbus();
        }
        /* 失能档或 SBUS 离线时保持停止，不回退到遥控 SWA。 */

        if (!Rail_SetDirection(&rail_controller, wanted))
        {
            (void)Rail_Stop(&rail_controller);
        }
        osDelay(RAIL_TASK_PERIOD_MS);
    }
}
