#include <math.h>
#include "config.h"
#include "control.h"

static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

void control_reset(control_t *c) {
    c->integ_l = 0.0f;
    c->integ_r = 0.0f;
}

// feedforward + PI on one wheel's speed, returns duty
static float wheel_velocity(float *integ, float v_cmd, float v_meas, float dt) {
    float err = v_cmd - v_meas;
    float ff = KV_DUTY_PER_MMPS * v_cmd;
    if (fabsf(v_cmd) > V_FF_DEADBAND_MMPS) {
        ff += copysignf(KS_DUTY, v_cmd);
    }

    float integ_new = clampf(*integ + KI_VEL * err * dt, -VEL_INTEG_MAX, VEL_INTEG_MAX);
    float duty = ff + KP_VEL * err + integ_new;

    // anti-windup: hold the integrator while the output is saturated in the same direction
    bool saturated = (duty > 1.0f && err > 0.0f) || (duty < -1.0f && err < 0.0f);
    if (!saturated) {
        *integ = integ_new;
    }
    return clampf(ff + KP_VEL * err + *integ, -1.0f, 1.0f);
}

/*
 * Cascade:
 *   schedule (position) P -> robot speed command
 *   heading PD            -> left/right speed difference
 *   per-wheel FF + PI     -> PWM duty
 * The heading D term uses the gyro rate directly, so there is no noisy
 * differentiation and slip-induced yaw is resisted immediately.
 * Yaw rate = (v_r - v_l)/track in either driving direction, so the same
 * heading law works while reversing.
 */
void control_step(control_t *c, const odom_t *o, float dir, float p_ref, float v_ref, float p,
                  float gyro_z, float dt, float *duty_l, float *duty_r) {
    float v_corr = clampf(KP_POS * (p_ref - p), -V_CORR_MAX_MMPS, V_CORR_MAX_MMPS);
    float v_cmd = clampf(v_ref + v_corr, -V_CREEP_MMPS, V_MAX_MMPS);

    float theta_ref = clampf(-dir * K_CROSS_RAD_PER_MM * o->y, -CROSS_MAX_RAD, CROSS_MAX_RAD);
    float dv = KP_HEAD * (theta_ref - o->theta) - KD_HEAD * gyro_z;
    float dv_lim = fmaxf(HEAD_DV_FRAC * fabsf(v_cmd), HEAD_DV_MIN_MMPS);
    dv = clampf(dv, -dv_lim, dv_lim);

    float v_l = dir * v_cmd - 0.5f * dv;
    float v_r = dir * v_cmd + 0.5f * dv;

    *duty_l = wheel_velocity(&c->integ_l, v_l, o->v_l, dt);
    *duty_r = wheel_velocity(&c->integ_r, v_r, o->v_r, dt);
}
