#ifndef LOGGER_H
#define LOGGER_H

#include <stdint.h>

// one row of run telemetry, kept in RAM so the run never waits on USB
typedef struct {
    float t;            // s since start
    float p_ref, p;     // scheduled / actual progress along the leg (mm)
    float v_ref, v;     // scheduled / actual speed (mm/s)
    float x, y;         // MP position (mm)
    float theta;        // heading (rad)
    float gyro_z;       // yaw rate (rad/s)
    float duty_l, duty_r;
    uint8_t state;
    uint8_t slipping;
} log_sample_t;

void logger_clear(void);
void logger_add(const log_sample_t *s);

// print the whole log as CSV over USB serial
void logger_dump(void);

#endif
