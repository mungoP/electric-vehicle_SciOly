#include <math.h>
#include <string.h>
#include "config.h"
#include "mt6701.h"
#include "odometry.h"

#define PI_F 3.14159265f
#define MM_PER_COUNT (PI_F * WHEEL_DIAMETER_MM / (float)MT6701_COUNTS_PER_REV)

void odom_reset(odom_t *o, int32_t count_l, int32_t count_r) {
    memset(o, 0, sizeof(*o));
    o->last_count_l = count_l;
    o->last_count_r = count_r;
}

/*
 * Heading comes mainly from the gyro. Encoder heading, (s_r - s_l)/track,
 * drifts with any left/right wheel size mismatch (0.5% over 10 m is ~19 deg),
 * so it only nudges the gyro estimate over a long time constant. When the
 * two disagree on yaw rate a wheel is slipping: trust only the gyro and
 * re-anchor the encoder heading so the slip doesn't pull theta afterwards.
 */
void odom_update(odom_t *o, int32_t count_l, int32_t count_r, float gyro_z, float dt) {
    float ds_l = (float)(count_l - o->last_count_l) * MM_PER_COUNT * WHEEL_SCALE_L;
    float ds_r = (float)(count_r - o->last_count_r) * MM_PER_COUNT * WHEEL_SCALE_R;
    o->last_count_l = count_l;
    o->last_count_r = count_r;
    o->s_l += ds_l;
    o->s_r += ds_r;

    float k_vel = dt / (VEL_FILTER_TAU_S + dt);
    o->v_l += k_vel * (ds_l / dt - o->v_l);
    o->v_r += k_vel * (ds_r / dt - o->v_r);
    o->v = 0.5f * (o->v_l + o->v_r);

    float omega_enc = (o->v_r - o->v_l) / TRACK_WIDTH_MM;
    if (fabsf(omega_enc - gyro_z) > SLIP_THRESH_RADPS) {
        o->slip_hold = SLIP_HOLD_S;
    } else if (o->slip_hold > 0.0f) {
        o->slip_hold -= dt;
    }
    o->slipping = o->slip_hold > 0.0f;

    float theta_prev = o->theta;
    o->theta += gyro_z * dt;

    float theta_enc = (o->s_r - o->s_l) / TRACK_WIDTH_MM;
    if (o->slipping) {
        o->enc_offset = o->theta - theta_enc;
    } else if (HEADING_FUSION_TAU_S > 0.0f) {
        float k_head = dt / (HEADING_FUSION_TAU_S + dt);
        o->theta += k_head * (theta_enc + o->enc_offset - o->theta);
    }

    float theta_mid = 0.5f * (theta_prev + o->theta);
    float ds = 0.5f * (ds_l + ds_r);
    o->x += ds * cosf(theta_mid);
    o->y += ds * sinf(theta_mid);
}
