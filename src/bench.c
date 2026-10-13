#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "bench.h"
#include "config.h"
#include "motor.h"
#include "mpu6050.h"
#include "pins.h"

#define PI_F 3.14159265f
#define MM_PER_COUNT (PI_F * WHEEL_DIAMETER_MM / (float)MT6701_COUNTS_PER_REV)

static void wait_for_button(const char *next_step) {
    printf("\nPress START for: %s\n", next_step);
    while (gpio_get(START_BTN_PIN)) sleep_ms(5);   // wait for press (active low)
    sleep_ms(30);
    while (!gpio_get(START_BTN_PIN)) sleep_ms(5);  // wait for release
    sleep_ms(30);
}

// run both motors for a while, return each wheel's average speed over the last `measure_s`
static void spin(mt6701_t *enc_l, mt6701_t *enc_r, float duty_l, float duty_r,
                 float total_s, float measure_s, float *v_l, float *v_r) {
    int ticks = (int)(total_s * LOOP_HZ);
    int measure_from = ticks - (int)(measure_s * LOOP_HZ);
    int32_t start_l = 0, start_r = 0;
    absolute_time_t start_t = get_absolute_time();
    absolute_time_t end_t = start_t;

    motor_set(duty_l, duty_r);
    for (int i = 0; i < ticks; i++) {
        mt6701_update(enc_l);
        mt6701_update(enc_r);
        end_t = get_absolute_time();
        if (i == measure_from) {
            start_l = enc_l->count;
            start_r = enc_r->count;
            start_t = end_t;
        }
        sleep_us(LOOP_PERIOD_US);
    }
    float elapsed = (float)absolute_time_diff_us(start_t, end_t) / 1e6f;
    motor_brake();

    *v_l = (float)(enc_l->count - start_l) * MM_PER_COUNT / elapsed;
    *v_r = (float)(enc_r->count - start_r) * MM_PER_COUNT / elapsed;
}

static void direction_test(mt6701_t *enc_l, mt6701_t *enc_r, bool left) {
    float v_l, v_r;
    spin(enc_l, enc_r, left ? 0.4f : 0.0f, left ? 0.0f : 0.4f, 1.0f, 0.5f, &v_l, &v_r);
    printf("  left wheel %.0f mm/s, right wheel %.0f mm/s\n", v_l, v_r);
    printf("  -> the %s wheel should have spun FORWARD and read a POSITIVE speed.\n",
           left ? "LEFT" : "RIGHT");
    printf("     wrong spin direction: flip %s_MOTOR_INVERT\n", left ? "LEFT" : "RIGHT");
    printf("     right direction but negative speed: flip ENC_%s_INVERT\n", left ? "LEFT" : "RIGHT");
}

static void gyro_test(void) {
    printf("  calibrating, keep still...\n");
    mpu6050_cal_reset();
    int16_t raw;
    bool done = false;
    while (!done) {
        if (mpu6050_read_gyro_z_raw(&raw)) done = mpu6050_cal_add(raw);
        sleep_us(LOOP_PERIOD_US);
    }
    printf("  now rotate the robot LEFT (counter-clockwise from above) for 5 s\n");
    for (int i = 0; i < 50; i++) {
        float rate = 0.0f;
        if (mpu6050_read_gyro_z_raw(&raw)) rate = mpu6050_gyro_z_radps(raw);
        printf("  yaw rate %+7.1f deg/s\n", rate * 57.2958f);
        sleep_ms(100);
    }
    printf("  -> turning left should read POSITIVE, otherwise flip GYRO_Z_INVERT\n");
}

static void duty_sweep(mt6701_t *enc_l, mt6701_t *enc_r) {
    printf("  duty,v_left_mmps,v_right_mmps\n");
    for (int d = 1; d <= 10; d++) {
        float duty = 0.1f * (float)d;
        float v_l, v_r;
        spin(enc_l, enc_r, duty, duty, 1.5f, 0.5f, &v_l, &v_r);
        printf("  %.1f,%.0f,%.0f\n", duty, v_l, v_r);
        sleep_ms(300);
    }
    printf("  -> fit duty = KV*v + KS: KV is the slope, KS the intercept.\n");
    printf("     The 1.0 duty speed is the top speed; set V_MAX_MMPS ~15%% below it.\n");
}

void bench_run(mt6701_t *enc_l, mt6701_t *enc_r) {
    // USB serial output is dropped until a terminal attaches, so wait for it
    while (!stdio_usb_connected()) sleep_ms(100);
    sleep_ms(500);
    printf("\n=== BENCH TEST MODE (wheels off the ground!) ===\n");
    for (;;) {
        wait_for_button("1/4 left motor direction");
        direction_test(enc_l, enc_r, true);
        wait_for_button("2/4 right motor direction");
        direction_test(enc_l, enc_r, false);
        wait_for_button("3/4 gyro sign");
        gyro_test();
        wait_for_button("4/4 duty -> speed sweep");
        duty_sweep(enc_l, enc_r);
    }
}
