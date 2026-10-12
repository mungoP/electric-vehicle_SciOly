#include <math.h>
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "config.h"
#include "motor.h"
#include "pins.h"

#define MOTOR_PWM_HZ 20000 // above hearing range

static uint32_t pwm_full; // level that keeps a pin high for the whole period

static void init_pwm_pin(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_wrap(&cfg, (uint16_t)(pwm_full - 1));
    pwm_init(pwm_gpio_to_slice_num(pin), &cfg, true);
}

void motor_init(void) {
    pwm_full = clock_get_hz(clk_sys) / MOTOR_PWM_HZ;
    if (pwm_full > 65535) pwm_full = 65535;

    // start braked so nothing moves while the slices come up
    gpio_init(IN1_PIN);
    gpio_init(IN2_PIN);
    gpio_init(IN3_PIN);
    gpio_init(IN4_PIN);
    init_pwm_pin(IN1_PIN);
    init_pwm_pin(IN2_PIN);
    init_pwm_pin(IN3_PIN);
    init_pwm_pin(IN4_PIN);
    motor_brake();
}

/*
 * Slow-decay drive: one input stays high, the other is PWM'd, so the motor
 * alternates between driving and braking. Speed tracks duty more linearly
 * than coasting (fast decay), which suits the velocity loop.
 *   forward: IN_A = 1, IN_B high for (1 - duty) of the period
 *   reverse: IN_B = 1, IN_A high for (1 - duty) of the period
 */
static void drive(uint pin_a, uint pin_b, float duty, int invert) {
    if (invert) duty = -duty;
    if (duty > 1.0f) duty = 1.0f;
    if (duty < -1.0f) duty = -1.0f;

    uint32_t mag = (uint32_t)(fabsf(duty) * (float)pwm_full + 0.5f);
    if (mag > pwm_full) mag = pwm_full;

    if (duty >= 0.0f) {
        pwm_set_gpio_level(pin_a, (uint16_t)pwm_full);
        pwm_set_gpio_level(pin_b, (uint16_t)(pwm_full - mag));
    } else {
        pwm_set_gpio_level(pin_a, (uint16_t)(pwm_full - mag));
        pwm_set_gpio_level(pin_b, (uint16_t)pwm_full);
    }
}

void motor_set(float left, float right) {
    drive(IN1_PIN, IN2_PIN, left, LEFT_MOTOR_INVERT);
    drive(IN3_PIN, IN4_PIN, right, RIGHT_MOTOR_INVERT);
}

void motor_brake(void) {
    pwm_set_gpio_level(IN1_PIN, (uint16_t)pwm_full);
    pwm_set_gpio_level(IN2_PIN, (uint16_t)pwm_full);
    pwm_set_gpio_level(IN3_PIN, (uint16_t)pwm_full);
    pwm_set_gpio_level(IN4_PIN, (uint16_t)pwm_full);
}

void motor_coast(void) {
    pwm_set_gpio_level(IN1_PIN, 0);
    pwm_set_gpio_level(IN2_PIN, 0);
    pwm_set_gpio_level(IN3_PIN, 0);
    pwm_set_gpio_level(IN4_PIN, 0);
}
