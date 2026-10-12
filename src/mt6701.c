#include "config.h"
#include "mt6701.h"

#define MT6701_REG_ANGLE_H 0x03 // angle[13:6], 0x04 holds angle[5:0] in bits 7:2

bool mt6701_read_raw(i2c_inst_t *i2c, uint8_t addr, uint16_t *raw) {
    uint8_t reg = MT6701_REG_ANGLE_H;
    uint8_t buffer[2];

    if (i2c_write_timeout_us(i2c, addr, &reg, 1, true, I2C_TIMEOUT_US) != 1) {
        return false;
    }
    if (i2c_read_timeout_us(i2c, addr, buffer, 2, false, I2C_TIMEOUT_US) != 2) {
        return false;
    }
    *raw = (uint16_t)((buffer[0] << 6) | (buffer[1] >> 2));
    return true;
}

bool mt6701_init(mt6701_t *enc, i2c_inst_t *i2c, uint8_t addr, bool invert) {
    enc->i2c = i2c;
    enc->addr = addr;
    enc->dir = invert ? -1 : 1;
    enc->count = 0;
    enc->last_raw = 0;
    return mt6701_read_raw(i2c, addr, &enc->last_raw);
}

bool mt6701_update(mt6701_t *enc) {
    uint16_t raw;
    if (!mt6701_read_raw(enc->i2c, enc->addr, &raw)) {
        return false;
    }

    // take the shortest way around the circle; valid while the wheel turns
    // less than half a revolution between reads
    int32_t delta = (int32_t)raw - (int32_t)enc->last_raw;
    if (delta > MT6701_COUNTS_PER_REV / 2) delta -= MT6701_COUNTS_PER_REV;
    if (delta < -MT6701_COUNTS_PER_REV / 2) delta += MT6701_COUNTS_PER_REV;

    enc->count += enc->dir * delta;
    enc->last_raw = raw;
    return true;
}
