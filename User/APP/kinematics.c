/**
 ******************************************************************************
 * @file    kinematics.c
 * @brief   6 轮差速车运动学解算（详见 kinematics.h 的符号约定）
 ******************************************************************************
 */

#include "kinematics.h"

static float clampf(float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return value;
}

static float wheel_track_width(unsigned char wheel_index)
{
    if (wheel_index == (unsigned char)WHEEL_LEFT_FRONT ||
        wheel_index == (unsigned char)WHEEL_RIGHT_FRONT)
    {
        return VEHICLE_TRACK_WIDTH_FRONT_M;
    }
    if (wheel_index == (unsigned char)WHEEL_LEFT_MID ||
        wheel_index == (unsigned char)WHEEL_RIGHT_MID)
    {
        return VEHICLE_TRACK_WIDTH_MID_M;
    }
    return VEHICLE_TRACK_WIDTH_REAR_M;
}

static float wheel_yaw_coefficient(unsigned char wheel_index)
{
    float side_sign = (wheel_index < (unsigned char)WHEEL_RIGHT_FRONT) ? -1.0f : +1.0f;
    return side_sign * 0.5f * wheel_track_width(wheel_index);
}

float kinematics_side_to_motor(float side_speed_ms, float dir)
{
    /* 由 v=omega*r 得 omega=v/r；dir 只处理左右电机的镜像安装。 */
    return side_speed_ms / WHEEL_RADIUS_M * dir;
}

float kinematics_motor_to_side(float motor_speed_radps, float dir)
{
    return motor_speed_radps * WHEEL_RADIUS_M * dir;
}

void kinematics_inverse_6wd(float v_x_ms, float omega_radps,
                            float wheel_speed_ms[WHEEL_COUNT])
{
    unsigned char i;

    for (i = 0u; i < (unsigned char)WHEEL_COUNT; i++)
    {
        wheel_speed_ms[i] = v_x_ms + wheel_yaw_coefficient(i) * omega_radps;
    }
}

void kinematics_forward_6wd(const float wheel_speed_ms[WHEEL_COUNT],
                            float *v_x_ms, float *omega_radps)
{
    float longitudinal_sum = 0.0f;
    float yaw_numerator = 0.0f;
    float yaw_denominator = 0.0f;
    unsigned char i;

    for (i = 0u; i < (unsigned char)WHEEL_COUNT; i++)
    {
        float coefficient = wheel_yaw_coefficient(i);
        longitudinal_sum += wheel_speed_ms[i];
        yaw_numerator += coefficient * wheel_speed_ms[i];
        yaw_denominator += coefficient * coefficient;
    }

    if (v_x_ms != 0)
    {
        *v_x_ms = longitudinal_sum / (float)WHEEL_COUNT;
    }
    if (omega_radps != 0)
    {
        *omega_radps = (yaw_denominator > 0.0f) ?
                       (yaw_numerator / yaw_denominator) : 0.0f;
    }
}

void kinematics_setpoints_6wd(float v_x_ms, float omega_radps,
                              float motor_speed_radps[WHEEL_COUNT])
{
    float wheel_speed[WHEEL_COUNT];
    float motor_limit;
    unsigned char i;

    kinematics_inverse_6wd(v_x_ms, omega_radps, wheel_speed);

    /* 协议层再留 5% 余量，避免浮点换算后落到 MIT 量程边界。 */
    motor_limit = G36_V_MAX * 0.95f;

    for (i = 0u; i < (unsigned char)WHEEL_COUNT; i++)
    {
        float side = clampf(wheel_speed[i], SIDE_SPEED_MAX_MS);
        float speed = kinematics_side_to_motor(side, g_wheel_motor_config[i].dir);
        motor_speed_radps[i] = clampf(speed, motor_limit);
    }
}

void kinematics_feedback_6wd(const float motor_vel_radps[WHEEL_COUNT],
                             float *v_left_ms, float *v_right_ms,
                             float *v_x_ms, float *omega_radps)
{
    float left_sum = 0.0f;
    float right_sum = 0.0f;
    float wheel_speed[WHEEL_COUNT];
    float v_left;
    float v_right;
    float v_x;
    float omega;
    unsigned char i;

    for (i = 0u; i < (unsigned char)WHEEL_COUNT; i++)
    {
        float side = kinematics_motor_to_side(motor_vel_radps[i],
                                              g_wheel_motor_config[i].dir);
        wheel_speed[i] = side;
        if (i <= (unsigned char)WHEEL_LEFT_REAR)
        {
            left_sum += side;
        }
        else
        {
            right_sum += side;
        }
    }

    /* 以左右三轮分别取平均，降低单轮反馈噪声。 */
    v_left  = left_sum / 3.0f;
    v_right = right_sum / 3.0f;
    kinematics_forward_6wd(wheel_speed, &v_x, &omega);

    if (v_left_ms != 0)
    {
        *v_left_ms = v_left;
    }
    if (v_right_ms != 0)
    {
        *v_right_ms = v_right;
    }
    if (v_x_ms != 0 || omega_radps != 0)
    {
        if (v_x_ms != 0)
        {
            *v_x_ms = v_x;
        }
        if (omega_radps != 0)
        {
            *omega_radps = omega;
        }
    }
}
