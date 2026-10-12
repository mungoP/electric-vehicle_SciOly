#ifndef MPU6050_H
#define MPU6050_H

#include <stdbool.h>
#include <stdint.h>
#include "hardware/i2c.h"

// wakes the IMU and sets the low-pass filter and gyro range; false on I2C failure
bool mpu6050_init(i2c_inst_t *i2c, uint8_t addr);

// raw gyro Z reading (only the yaw axis is used while driving)
bool mpu6050_read_gyro_z_raw(int16_t *raw);

// bias-corrected yaw rate, rad/s, positive = turning left (CCW from above)
float mpu6050_gyro_z_radps(int16_t raw);

// start a fresh bias calibration
void mpu6050_cal_reset(void);

// feed one sample while the robot sits still; true once the bias is locked in.
// restarts by itself if the samples show the robot moved.
bool mpu6050_cal_add(int16_t raw);

// slow bias refinement while waiting for the start button
void mpu6050_bias_track(int16_t raw);

float mpu6050_bias(void);

#endif
