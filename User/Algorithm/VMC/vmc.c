#include "vmc.h"

#include <float.h>
#include <math.h>

static uint8_t vmc_is_finite(float value)
{
    return (uint8_t)((value == value) && (value <= FLT_MAX) && (value >= -FLT_MAX));
}

static uint8_t vmc_parameters_valid(const vmc_joint_parameters_t *parameters)
{
    if (parameters == 0 ||
        !vmc_is_finite(parameters->kp_nm_per_rad) ||
        !vmc_is_finite(parameters->kd_nms_per_rad) ||
        !vmc_is_finite(parameters->gravity_ff_max_nm) ||
        !vmc_is_finite(parameters->gravity_ff_direction) ||
        !vmc_is_finite(parameters->gravity_reference_abs_rad) ||
        !vmc_is_finite(parameters->torque_limit_nm) ||
        !vmc_is_finite(parameters->joint_min_rad) ||
        !vmc_is_finite(parameters->joint_max_rad) ||
        !vmc_is_finite(parameters->joint_ref_rad))
    {
        return 0u;
    }

    return (uint8_t)(parameters->kp_nm_per_rad >= 0.0f &&
                     parameters->kd_nms_per_rad >= 0.0f &&
                     (parameters->gravity_ff_direction == 1.0f ||
                      parameters->gravity_ff_direction == -1.0f) &&
                     parameters->gravity_reference_abs_rad >= 0.0f &&
                     parameters->torque_limit_nm > 0.0f &&
                     parameters->joint_min_rad < parameters->joint_max_rad &&
                     parameters->joint_ref_rad >= parameters->joint_min_rad &&
                     parameters->joint_ref_rad <= parameters->joint_max_rad);
}

uint8_t vmc_calculate_joint_torque(const vmc_joint_parameters_t *parameters,
                                   float joint_position_rad,
                                   float joint_velocity_radps,
                                   vmc_joint_output_t *output)
{
    vmc_joint_output_t result;
    float torque;

    if (output == 0 || !vmc_parameters_valid(parameters) ||
        !vmc_is_finite(joint_position_rad) ||
        !vmc_is_finite(joint_velocity_radps))
    {
        return 0u;
    }

    result.joint_error_rad = parameters->joint_ref_rad - joint_position_rad;
    result.gravity_ff_nm = parameters->gravity_ff_direction *
        parameters->gravity_ff_max_nm *
        cosf(parameters->gravity_reference_abs_rad - fabsf(joint_position_rad));
    result.unsaturated_torque_nm =
        parameters->kp_nm_per_rad * result.joint_error_rad -
        parameters->kd_nms_per_rad * joint_velocity_radps +
        result.gravity_ff_nm;
    torque = result.unsaturated_torque_nm;

    if (torque > parameters->torque_limit_nm)
    {
        torque = parameters->torque_limit_nm;
    }
    else if (torque < -parameters->torque_limit_nm)
    {
        torque = -parameters->torque_limit_nm;
    }

    /* At either software limit, block torque that would push farther outward. */
    if ((joint_position_rad <= parameters->joint_min_rad && torque < 0.0f) ||
        (joint_position_rad >= parameters->joint_max_rad && torque > 0.0f))
    {
        torque = 0.0f;
    }

    if (!vmc_is_finite(result.gravity_ff_nm) ||
        !vmc_is_finite(result.unsaturated_torque_nm) || !vmc_is_finite(torque))
    {
        return 0u;
    }

    result.motor_torque_nm = torque;
    *output = result;
    return 1u;
}
