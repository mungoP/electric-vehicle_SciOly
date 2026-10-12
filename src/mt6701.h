#ifndef MT6701_H
#define MT6701_H

#include <stdbool.h>
#include <stdint.h>
#include "hardware/i2c.h"

#define MT6701_COUNTS_PER_REV 16384

// one MT6701 magnetic encoder, unwrapped into a multi-turn count
typedef struct {
    i2c_inst_t *i2c;
    uint8_t addr;
    int8_t dir;         // +1 or -1 so forward wheel motion counts up
    uint16_t last_raw;
    int32_t count;      // total counts since init
} mt6701_t;

// read the 14-bit angle (0..16383); false if the I2C transfer failed
bool mt6701_read_raw(i2c_inst_t *i2c, uint8_t addr, uint16_t *raw);

// first read primes the unwrap; false if the encoder did not respond
bool mt6701_init(mt6701_t *enc, i2c_inst_t *i2c, uint8_t addr, bool invert);

// read and accumulate; count is left unchanged if the read fails
bool mt6701_update(mt6701_t *enc);

#endif
