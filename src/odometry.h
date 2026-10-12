#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <stdbool.h>
#include <stdint.h>

// robot pose from the wheel encoders and gyro; x points down the track from the start point
typedef struct {
    int32_t last_count_l;
    int32_t last_count_r;
    float s_l, s_r;     // distance travelled by each wheel (mm)
    float v_l, v_r;     // filtered wheel speeds (mm/s)
    float v;            // filtered robot speed (mm/s)
    float theta;        // fused heading (rad), positive = left
    float enc_offset;   // re-anchors encoder heading after slip
    float x, y;         // MP position (mm)
    bool slipping;
    float slip_hold;
} odom_t;

// zero the pose at the current encoder counts
void odom_reset(odom_t *o, int32_t count_l, int32_t count_r);

// advance by one tick
void odom_update(odom_t *o, int32_t count_l, int32_t count_r, float gyro_z, float dt);

#endif
