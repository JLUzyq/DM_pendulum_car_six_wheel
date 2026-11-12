#ifndef __VMC_H
#define __VMC_H

#include <stdint.h>

typedef struct
{
    float kp_nm_per_rad;       /* 关节虚拟弹簧刚度 N*m/rad */
    float kd_nms_per_rad;      /* 关节虚拟阻尼 N*m*s/rad */
    float gravity_ff_max_nm;   /* 摆臂水平时的重力前馈幅值 N*m */
    float gravity_ff_direction; /* 重力前馈方向：+1 或 -1 */
    float gravity_reference_abs_rad; /* 重力前馈公式中的水平参考角绝对值 */
    float torque_limit_nm;     /* 输出力矩限幅 N*m */
    float joint_min_rad;       /* 关节下限 rad */
    float joint_max_rad;       /* 关节上限 rad */
    float joint_ref_rad;       /* 平衡目标角 rad */
} vmc_joint_parameters_t;

typedef struct
{
    float joint_error_rad;
    float gravity_ff_nm;       /* 当前角度下的重力前馈力矩幅值 N*m */
    float unsaturated_torque_nm;
    float motor_torque_nm;
} vmc_joint_output_t;

/* 关节空间 VMC：tau=Kp(q_ref-q)-Kd*q_dot+G_ff，并执行范围/力矩限幅。 */
uint8_t vmc_calculate_joint_torque(const vmc_joint_parameters_t *parameters,
                                   float joint_position_rad,
                                   float joint_velocity_radps,
                                   vmc_joint_output_t *output);

#endif /* __VMC_H */
