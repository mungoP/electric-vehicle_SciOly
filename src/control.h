#ifndef CONTROL_H
#define CONTROL_H

#include "odometry.h"

typedef struct {
    float integ_l;
    float integ_r;
} control_t;

// clear the velocity integrators (call at the start of each leg)
void control_reset(control_t *c);

/*
 * One control step for a leg.
 *   dir:   +1 driving forward, -1 reversing
 *   p_ref, v_ref: scheduled progress (mm) and speed (mm/s) along the leg
 *   p:     actual progress along the leg (mm)
 * Outputs signed motor duties.
 */
void control_step(control_t *c, const odom_t *o, float dir, float p_ref, float v_ref, float p,
                  float gyro_z, float dt, float *duty_l, float *duty_r);

#endif
