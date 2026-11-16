/**
 ******************************************************************************
 * @file    chassis_task.c
 * @brief   6 轮差速移动车底盘任务实现（详见 chassis_task.h）
 ******************************************************************************
 */

#include "chassis_task.h"
#include "kinematics.h"
#include "vmc.h"
#include "fdcan.h"
#include "cmsis_os.h"
#include "rc_sbus.h"
#include "Chaiss_Communication.h"

extern INS_t INS;

chassis_t chassis_move;

static float clampf(float value, float limit)
{
    /* 对称限幅：把 value 限制在 [-limit, +limit]。 */
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

static float ramp_to(float current, float target, float max_step)
{
    float delta = target - current;
    if (delta > max_step)
    {
        delta = max_step;
    }
    else if (delta < -max_step)
    {
        delta = -max_step;
    }
    return current + delta;
}

/* 按电机配置表选择所在的 CAN 总线句柄 */
static hcan_t *bus_handle(motor_can_bus_e bus)
{
    return (bus == MOTOR_CAN1) ? (hcan_t *)&hfdcan1 : (hcan_t *)&hfdcan2;
}

static void suspension_vmc_update(void)
{
    static float filtered_joint_ref[SUSPENSION_MOTOR_COUNT];
    static uint8_t ref_filter_initialized[SUSPENSION_MOTOR_COUNT];
    static uint8_t startup_recovery_active[SUSPENSION_MOTOR_COUNT];
    unsigned char i;
    const uint32_t now_ms = HAL_GetTick();
    const float dt_s = (float)CHASSIS_TASK_PERIOD_MS * 0.001f;
    const float alpha = dt_s / (SUSPENSION_VMC_REF_TAU_S + dt_s);
    const float pose_input = rc_sbus.online ?
        rc_sbus_channel_norm(g_sbus_ch_map.suspension_pose) : 0.0f;

    for (i = 0u; i < (unsigned char)SUSPENSION_MOTOR_COUNT; i++)
    {
        const suspension_motor_config_t *config = &g_suspension_motor_config[i];
        const HM_Motor_t *motor = &chassis_move.suspension_motor[i];
        float target_joint_ref;
        vmc_joint_parameters_t parameters = {
            .kp_nm_per_rad = SUSPENSION_VMC_KP_NM_PER_RAD,
            .kd_nms_per_rad = SUSPENSION_VMC_KD_NMS_PER_RAD,
            .gravity_ff_max_nm = SUSPENSION_VMC_G_FF_MAX_NM,
            .gravity_ff_direction = config->gravity_ff_direction,
            .gravity_reference_abs_rad = SUSPENSION_HORIZONTAL_ABS_RAD,
            .torque_limit_nm = SUSPENSION_VMC_TORQUE_MAX_NM,
            .joint_min_rad = config->joint_min_rad,
            .joint_max_rad = config->joint_max_rad,
            .joint_ref_rad = filtered_joint_ref[i]
        };
        vmc_joint_output_t output;

        if (motor->last_rx_ms == 0u ||
            (now_ms - motor->last_rx_ms) > SUSPENSION_FEEDBACK_TIMEOUT_MS)
        {
            ref_filter_initialized[i] = 0u;
            hm_motor_send_relax(bus_handle(config->bus), config->can_id);
            continue;
        }

        if (ref_filter_initialized[i] == 0u)
        {
            filtered_joint_ref[i] = motor->pos;
            ref_filter_initialized[i] = 1u;
            startup_recovery_active[i] = 1u;
        }

        /* CH2 中位保持默认平衡角；正向趋向 0 rad，负向远离水平位。 */
        if (i == 0u)
        {
            target_joint_ref = (pose_input >= 0.0f) ?
                (-0.4f * (1.0f - pose_input)) :
                (-0.4f + 0.6f * pose_input);
        }
        else
        {
            target_joint_ref = (pose_input >= 0.0f) ?
                (0.4f * (1.0f - pose_input)) :
                (0.4f - 0.6f * pose_input);
        }
        if (startup_recovery_active[i] != 0u)
        {
            /* 仅上电后先平滑回默认平衡角，再交由遥控器直接调节。 */
            filtered_joint_ref[i] += alpha *
                (config->joint_ref_rad - filtered_joint_ref[i]);
            if ((filtered_joint_ref[i] - config->joint_ref_rad < 0.005f) &&
                (filtered_joint_ref[i] - config->joint_ref_rad > -0.005f))
            {
                filtered_joint_ref[i] = config->joint_ref_rad;
                startup_recovery_active[i] = 0u;
            }
        }
        else
        {
            filtered_joint_ref[i] = target_joint_ref;
        }
        parameters.joint_ref_rad = filtered_joint_ref[i];

        if (!vmc_calculate_joint_torque(&parameters, motor->pos, motor->vel, &output))
        {
            ref_filter_initialized[i] = 0u;
            hm_motor_send_relax(bus_handle(config->bus), config->can_id);
            continue;
        }

        /* MIT torque mode: position/velocity gains are zero; VMC provides T_ff. */
        hm_motor_send_mit(bus_handle(config->bus), config->can_id,
                          0.0f, 0.0f, 0.0f, 0.0f,
                          output.motor_torque_nm);
    }
}

void Chassis_init(chassis_t *chassis)
{
    unsigned char i;
    unsigned char j;

    /* 数组顺序必须与 vehicle_config.h 中 wheel_index_e 完全一致。 */
    for (i = 0u; i < (unsigned char)WHEEL_COUNT; i++)
    {
        chassis->motor[i].id = g_wheel_motor_config[i].can_id;
    }
    for (i = 0u; i < (unsigned char)SUSPENSION_MOTOR_COUNT; i++)
    {
        chassis->suspension_motor[i].id = g_suspension_motor_config[i].can_id;
    }

    /* 上电后连续发送使能帧，保证 6 个电机可靠进入 MIT 模式 */
    for (j = 0u; j < 5u; j++)
    {
        for (i = 0u; i < (unsigned char)WHEEL_COUNT; i++)
        {
            hm_motor_enable(bus_handle(g_wheel_motor_config[i].bus),
                            g_wheel_motor_config[i].can_id);
            osDelay(1);
        }
    }

    /* 使能左右前摆臂关节（CAN1 ID7 / CAN2 ID8）。 */
    for (j = 0u; j < 5u; j++)
    {
        for (i = 0u; i < (unsigned char)SUSPENSION_MOTOR_COUNT; i++)
        {
            hm_motor_enable(bus_handle(g_suspension_motor_config[i].bus),
                            g_suspension_motor_config[i].can_id);
            osDelay(1);
        }
    }
}

void Chassis_task(void)
{
    float motor_vel[WHEEL_COUNT];
    float motor_speed[WHEEL_COUNT];
    float v_applied = 0.0f; /* 经过斜坡限制后的线速度指令 */
    float yaw_rate_applied = 0.0f;
    float yaw_rate_imu_filt = 0.0f;
    float yaw_rate_integral = 0.0f;
    Chaiss_YawRateTuningConfig_t yaw_rate_tuning;
    const float dt = (float)CHASSIS_TASK_PERIOD_MS / 1000.0f;
    unsigned char i;

    /* 等待 IMU 收敛 */
    while (INS.ins_flag == 0u)
    {
        osDelay(1);
    }

    Chassis_init(&chassis_move);

    while (1)
    {
        /* 第 1 步：读取 IMU。总航向角不会在 +/-PI 处跳变。 */
        chassis_move.yaw   = INS.YawTotalAngle;
        yaw_rate_imu_filt += IMU_YAW_RATE_LPF_ALPHA *
                             (INS.Gyro[2] * IMU_YAW_RATE_SIGN -
                              yaw_rate_imu_filt);
        chassis_move.d_yaw = yaw_rate_imu_filt;
        Chaiss_Communication_GetYawRateTuning(&yaw_rate_tuning);

        /* 第 2 步：六个电机反馈经轮径和安装方向换算为车体速度。 */
        for (i = 0u; i < (unsigned char)WHEEL_COUNT; i++)
        {
            motor_vel[i] = chassis_move.motor[i].vel;
        }
        kinematics_feedback_6wd(motor_vel,
                                &chassis_move.v_left, &chassis_move.v_right,
                                &chassis_move.v, &chassis_move.omega);

        {
            chassis_command_t command;
            uint8_t command_ok = chassis_command_read(&command);
            uint32_t command_age = HAL_GetTick() - command.update_ms;

            if ((command_ok != 0u) &&
                (command.enabled != 0u) &&
                (command_age <= CHASSIS_CMD_TIMEOUT_MS))
            {
                chassis_move.start_flag = 1u;
                chassis_move.command_source = command.source;

                if (command.mode == CHASSIS_MODE_HEADING_LOCK)
                {
                    /* Capture the current continuous yaw only on mode entry. */
                    if (chassis_move.heading_lock_active == 0u)
                    {
                        chassis_move.heading_lock_target = chassis_move.yaw;
                    }
                    chassis_move.heading_lock_active = 1u;
                    chassis_move.heading_lock_error =
                        chassis_move.heading_lock_target - chassis_move.yaw;
                    chassis_move.v_set = 0.0f;
                    chassis_move.yaw_rate_set = clampf(
                        HEADING_LOCK_KP * chassis_move.heading_lock_error,
                        HEADING_LOCK_RETURN_RATE_MAX_RADPS);
                }
                else
                {
                    chassis_move.heading_lock_active = 0u;
                    chassis_move.heading_lock_error = 0.0f;
                    chassis_move.v_set =
                        clampf(command.linear_x, V_CMD_MAX_MS);
                    chassis_move.yaw_rate_set =
                        clampf(command.angular_z, YAW_RATE_MAX_RADPS);
                }
            }
            else
            {
                chassis_move.start_flag = 0u;
                chassis_move.v_set = 0.0f;
                chassis_move.yaw_rate_set = 0.0f;
                chassis_move.heading_lock_active = 0u;
                chassis_move.heading_lock_error = 0.0f;
                chassis_move.command_source = CHASSIS_CMD_NONE;
            }
        }

        if (chassis_move.start_flag != 0u)
        {
            float yaw_rate_correction;

            /*
             * 第 3 步：角速度运动学前馈 + IMU 反馈。
             * 前馈保证在误差为零时仍能维持目标转速；PI 只补偿打滑、
             * 左右负载差和模型误差。KI 默认关闭，先在低速下调 KP。
             */
            v_applied = ramp_to(v_applied, chassis_move.v_set,
                                V_CMD_RAMP_MPS2 * dt);
            yaw_rate_applied = ramp_to(yaw_rate_applied,
                                       chassis_move.yaw_rate_set,
                                       YAW_RATE_RAMP_RADPS2 * dt);

            chassis_move.yaw_rate_error =
                yaw_rate_applied - yaw_rate_imu_filt;
            yaw_rate_integral += chassis_move.yaw_rate_error * dt;
            yaw_rate_integral = clampf(yaw_rate_integral,
                                       YAW_RATE_INTEGRAL_LIMIT);
            yaw_rate_correction =
                yaw_rate_tuning.kp * chassis_move.yaw_rate_error +
                yaw_rate_tuning.ki * yaw_rate_integral;
            yaw_rate_correction =
                clampf(yaw_rate_correction,
                       YAW_RATE_CORRECTION_MAX_RADPS);
            chassis_move.yaw_rate_control =
                yaw_rate_applied + yaw_rate_correction;

            /*
             * 第 4 步：运动学分配。各轴按自身轮距计算差速，再除以轮半径
             * 得到电机 rad/s，最后乘每台电机的安装方向 dir。
             */
            kinematics_setpoints_6wd(v_applied,
                                     chassis_move.yaw_rate_control,
                                     motor_speed);

            for (i = 0u; i < (unsigned char)WHEEL_COUNT; i++)
            {
                hm_motor_send_velocity(bus_handle(g_wheel_motor_config[i].bus),
                                       g_wheel_motor_config[i].can_id,
                                       motor_speed[i]);
            }
        }
        else
        {
            /*
             * 未使能或指令超时时发送全零 MIT 指令让电机放松，并清除
             * 速度斜坡及 PI 状态，避免重新接管时突然运动。
             */
            v_applied = 0.0f;
            yaw_rate_applied = 0.0f;
            yaw_rate_integral = 0.0f;
            chassis_move.yaw_rate_error = 0.0f;
            chassis_move.yaw_rate_control = 0.0f;
            chassis_move.v_set = 0.0f;
            chassis_move.yaw_rate_set = 0.0f;

            for (i = 0u; i < (unsigned char)WHEEL_COUNT; i++)
            {
                hm_motor_send_relax(bus_handle(g_wheel_motor_config[i].bus),
                                    g_wheel_motor_config[i].can_id);
            }
        }

        /* 摆臂平衡不依赖底盘行驶指令，始终由关节空间 VMC 闭环控制。 */
        suspension_vmc_update();

        /* 任务上下文非阻塞仲裁配置确认与可选角速度遥测。 */
        Chaiss_Communication_ProcessTx(HAL_GetTick(),
                                       yaw_rate_applied,
                                       yaw_rate_imu_filt);

        osDelay(CHASSIS_TASK_PERIOD_MS);
    }
}
