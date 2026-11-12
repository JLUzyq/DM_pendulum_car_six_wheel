#ifndef __KINEMATICS_H
#define __KINEMATICS_H

/**
 ******************************************************************************
 * @file    kinematics.h
 * @brief   6 轮差速车运动学解算
 *
 *          车体模型：左右两侧各 3 个车轮，各轴可以使用不同轮距。
 *          正运动学：由六个车轮接地速度和对应轴轮距估计 v_x 和 omega。
 *          逆运动学：由车体 v_x 与 omega 分别计算 6 个车轮的角速度指令。
 *
 *          符号约定（俯视）：
 *            - 车体前进方向为 x 轴正方向，v_x > 0 为前进；
 *            - 偏航角 yaw 逆时针为正，omega > 0 为左转；
 *            - v_left / v_right 为左/右侧车轮的接地线速度（m/s）；
 *            - 左轮速度 = v_x - omega * 该轴轮距 / 2；
 *            - 右轮速度 = v_x + omega * 该轴轮距 / 2。
 ******************************************************************************
 */

#include "vehicle_config.h"

/* 单侧线速度 (m/s) -> 电机输出轴角速度 (rad/s)，含安装方向 */
float kinematics_side_to_motor(float side_speed_ms, float dir);

/* 电机输出轴角速度 (rad/s) -> 单侧线速度 (m/s)，含安装方向 */
float kinematics_motor_to_side(float motor_speed_radps, float dir);

/* 逆运动学：车体线速度和偏航角速度 -> 六个轮子的接地线速度 (m/s)。 */
void kinematics_inverse_6wd(float v_x_ms, float omega_radps,
                            float wheel_speed_ms[WHEEL_COUNT]);

/* 正运动学：六轮接地线速度 -> 车体 v_x 与偏航角速度 omega */
void kinematics_forward_6wd(const float wheel_speed_ms[WHEEL_COUNT],
                            float *v_x_ms, float *omega_radps);

/* 逆解并展开到 6 个电机：
 * 输入  v_x_ms     车体系线速度指令 (m/s)
 *       omega_radps 车体偏航角速度指令 (rad/s)
 * 输出  motor_speed_radps[WHEEL_COUNT]  按 wheel_index_e 顺序的电机角速度 (rad/s)
 * 内部完成：分轴轮速计算、单轮限幅、轮径换算、逐电机方向符号及电机量程限幅。 */
void kinematics_setpoints_6wd(float v_x_ms, float omega_radps,
                              float motor_speed_radps[WHEEL_COUNT]);

/* 由 6 个电机的反馈角速度解算车体状态：
 * 输入  motor_vel_radps[WHEEL_COUNT]  按 wheel_index_e 顺序的电机反馈角速度 (rad/s)
 * 输出  v_left_ms / v_right_ms / v_x_ms / omega_radps（不需要的项可传 0） */
void kinematics_feedback_6wd(const float motor_vel_radps[WHEEL_COUNT],
                             float *v_left_ms, float *v_right_ms,
                             float *v_x_ms, float *omega_radps);

#endif /* __KINEMATICS_H */
