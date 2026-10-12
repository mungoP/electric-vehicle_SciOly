#include <math.h>
#include <stdio.h>
#include "pico/time.h"
#include "config.h"
#include "mpu6050.h"

#define REG_SMPLRT_DIV   0x19
#define REG_CONFIG       0x1A
#define REG_GYRO_CONFIG  0x1B
#define REG_GYRO_ZOUT_H  0x47
#define REG_PWR_MGMT_1   0x6B
#define REG_WHO_AM_I     0x75

#define DEG_TO_RAD 0.0174532925f

static i2c_inst_t *imu_i2c;
static uint8_t imu_addr;
static float gyro_z_bias; // raw LSB, kept as float so it isn't truncated

static int64_t cal_sum;
static int64_t cal_sum_sq;
static int cal_n;

static bool write_reg(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    return i2c_write_timeout_us(imu_i2c, imu_addr, buf, 2, false, I2C_TIMEOUT_US) == 2;
}

static bool read_regs(uint8_t reg, uint8_t *out, size_t len) {
    if (i2c_write_timeout_us(imu_i2c, imu_addr, &reg, 1, true, I2C_TIMEOUT_US) != 1) {
        return false;
    }
    return i2c_read_timeout_us(imu_i2c, imu_addr, out, len, false, I2C_TIMEOUT_US) == (int)len;
}

bool mpu6050_init(i2c_inst_t *i2c, uint8_t addr) {
    imu_i2c = i2c;
    imu_addr = addr;

    bool ok = write_reg(REG_PWR_MGMT_1, 0x80); // device reset
    sleep_ms(100);
    ok = ok && write_reg(REG_PWR_MGMT_1, 0x01);  // wake, clock from gyro X PLL (more stable)
    sleep_ms(10);
    ok = ok && write_reg(REG_CONFIG, 0x03);      // DLPF ~44 Hz, replaces the old moving average
    ok = ok && write_reg(REG_SMPLRT_DIV, 0x00);  // 1 kHz output
    ok = ok && write_reg(REG_GYRO_CONFIG, 0x08); // +-500 dps, 65.5 LSB/dps

    uint8_t who = 0;
    ok = ok && read_regs(REG_WHO_AM_I, &who, 1);
    if (ok && who != 0x68) {
        // clones report other IDs but usually behave the same
        printf("MPU6050 WHO_AM_I = 0x%02x (expected 0x68), continuing\n", who);
    }

    gyro_z_bias = 0.0f;
    mpu6050_cal_reset();
    return ok;
}

bool mpu6050_read_gyro_z_raw(int16_t *raw) {
    uint8_t buf[2];
    if (!read_regs(REG_GYRO_ZOUT_H, buf, 2)) {
        return false;
    }
    *raw = (int16_t)((buf[0] << 8) | buf[1]);
    return true;
}

float mpu6050_gyro_z_radps(int16_t raw) {
    float rate = ((float)raw - gyro_z_bias) / GYRO_LSB_PER_DPS * DEG_TO_RAD;
    return GYRO_Z_INVERT ? -rate : rate;
}

void mpu6050_cal_reset(void) {
    cal_sum = 0;
    cal_sum_sq = 0;
    cal_n = 0;
}

bool mpu6050_cal_add(int16_t raw) {
    cal_sum += raw;
    cal_sum_sq += (int64_t)raw * raw;
    cal_n++;
    if (cal_n < CAL_SAMPLES) {
        return false;
    }

    float mean = (float)cal_sum / (float)cal_n;
    float var = (float)cal_sum_sq / (float)cal_n - mean * mean;
    float std = sqrtf(var > 0.0f ? var : 0.0f);
    mpu6050_cal_reset();

    if (std > CAL_STILL_STD_LSB) {
        printf("Gyro calibration: robot moved (std %.1f LSB), restarting\n", std);
        return false;
    }
    gyro_z_bias = mean;
    printf("Gyro calibration done: bias %.2f LSB, noise %.2f LSB\n", mean, std);
    return true;
}

void mpu6050_bias_track(int16_t raw) {
    float err = (float)raw - gyro_z_bias;
    if (fabsf(err) < BIAS_TRACK_STILL_LSB) {
        gyro_z_bias += BIAS_TRACK_ALPHA * err;
    }
}

float mpu6050_bias(void) {
    return gyro_z_bias;
}
