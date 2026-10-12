#include <math.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "bench.h"
#include "config.h"
#include "control.h"
#include "logger.h"
#include "motor.h"
#include "mpu6050.h"
#include "mt6701.h"
#include "odometry.h"
#include "pins.h"
#include "profile.h"

typedef enum {
    ST_CALIBRATE,
    ST_ARMED,
    ST_RUN_FWD,
    ST_REVERSE_PAUSE,
    ST_RUN_REV,
    ST_FINISHED,
    ST_FAULT,
} run_state_t;

static run_state_t state;
static const char *fault_reason = "";

static mt6701_t enc_left, enc_right;
static odom_t odom;
static control_t ctrl;
static course_t course;
static profile_t fwd_profile, rev_profile;
static plan_result_t plan_result;

static absolute_time_t t_run_start;   // start button press
static absolute_time_t t_leg_start;   // start of the current leg's profile
static absolute_time_t t_pause_start;
static float leg_start_x;             // x the current leg's progress is measured from
static float leg_dir;                 // +1 forward, -1 reverse
static float run_time_s;

static int sensor_fail_count;
static float last_gyro_z;
static uint32_t tick_count;
static uint32_t overrun_count;

static uint8_t btn_history;
static bool btn_down = true; // assume held at boot so a press must start from released

static float seconds_since(absolute_time_t from, absolute_time_t now) {
    return (float)absolute_time_diff_us(from, now) / 1e6f;
}

// debounced press edge: 4 identical samples (20 ms) to change state
static bool button_pressed_edge(void) {
    btn_history = (uint8_t)((btn_history << 1) | (gpio_get(START_BTN_PIN) ? 0 : 1));
    if (!btn_down && (btn_history & 0x0F) == 0x0F) {
        btn_down = true;
        return true;
    }
    if (btn_down && (btn_history & 0x0F) == 0x00) {
        btn_down = false;
    }
    return false;
}

static void init_i2c_buses(void) {
    // bus 0: left encoder + IMU
    i2c_init(i2c0, I2C_BAUD);
    gpio_set_function(ENC_LEFT_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(ENC_LEFT_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(ENC_LEFT_SDA_PIN);
    gpio_pull_up(ENC_LEFT_SCL_PIN);

    // bus 1: right encoder
    i2c_init(i2c1, I2C_BAUD);
    gpio_set_function(ENC_RIGHT_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(ENC_RIGHT_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(ENC_RIGHT_SDA_PIN);
    gpio_pull_up(ENC_RIGHT_SCL_PIN);
}

static void enter_fault(const char *reason) {
    motor_brake();
    fault_reason = reason;
    state = ST_FAULT;
    printf("FAULT: %s\n", reason);
}

static void plan_and_arm(void) {
    course = course_from_config();
    float budget = TARGET_TIME_S - T_PAUSE_S - T_END_MARGIN_S;
    plan_result = plan_run(&fwd_profile, &rev_profile, &course, budget);

    printf("\nTarget %.0f mm, time %.1f s, bottle line %.0f mm past target, bottle at %.0f mm\n",
           course.target_mm, TARGET_TIME_S, course.line_from_target_mm, course.bottle_start_mm);
    printf("Plan: forward %.0f mm in %.2f s, reverse %.0f mm in %.2f s, cruise %.0f mm/s\n",
           fwd_profile.length, fwd_profile.duration, rev_profile.length, rev_profile.duration,
           fwd_profile.cruise);
    if (plan_result == PLAN_TOO_FAST) {
        printf("WARNING: needs more than V_MAX, will finish ~%.2f s late\n",
               fwd_profile.duration + rev_profile.duration - budget);
    } else if (plan_result == PLAN_TOO_SLOW) {
        printf("WARNING: slower than V_MIN_CRUISE, will finish early\n");
    }
    printf("ARMED - press START\n");
    state = ST_ARMED;
}

static void start_run(absolute_time_t now) {
    odom_reset(&odom, enc_left.count, enc_right.count);
    control_reset(&ctrl);
    logger_clear();
    t_run_start = now;
    t_leg_start = now;
    leg_start_x = 0.0f;
    leg_dir = 1.0f;
    state = ST_RUN_FWD;
}

static void start_pause(absolute_time_t now) {
    motor_brake();
    control_reset(&ctrl);
    t_pause_start = now;
    state = ST_REVERSE_PAUSE;

    // re-plan the reverse leg from where we actually stopped, with the time actually left
    float to_target = odom.x - course.target_mm;
    leg_dir = (to_target >= 0.0f) ? -1.0f : 1.0f;
    leg_start_x = odom.x;
    float budget = TARGET_TIME_S - T_END_MARGIN_S - (seconds_since(t_run_start, now) + T_PAUSE_S);
    if (budget < 0.5f) budget = 0.5f;
    plan_reverse(&rev_profile, fabsf(to_target), budget);
}

static void finish_run(absolute_time_t now) {
    motor_brake();
    run_time_s = seconds_since(t_run_start, now);
    state = ST_FINISHED;
    printf("\nFINISHED: run time %.3f s (target %.1f), x %.1f mm (target %.0f, error %+.1f), "
           "y %+.1f mm, heading %+.2f deg, loop overruns %lu\n",
           run_time_s, TARGET_TIME_S, odom.x, course.target_mm, odom.x - course.target_mm,
           odom.y, odom.theta * 57.2958f, (unsigned long)overrun_count);
    logger_dump();
    printf("Press START to print the log again. Power cycle for a new run.\n");
}

// follow the current leg's profile; returns seconds since the leg started
static float drive_leg(profile_t *prof, absolute_time_t now, float dt, float gyro_z) {
    float t = seconds_since(t_leg_start, now);
    float p_ref, v_ref;
    profile_sample(prof, t, &p_ref, &v_ref);
    float p = (odom.x - leg_start_x) * leg_dir;

    float duty_l, duty_r;
    control_step(&ctrl, &odom, leg_dir, p_ref, v_ref, p, gyro_z, dt, &duty_l, &duty_r);
    motor_set(duty_l, duty_r);

    if (tick_count % LOG_DIVIDER == 0) {
        log_sample_t s = {
            .t = seconds_since(t_run_start, now),
            .p_ref = p_ref, .p = p, .v_ref = v_ref, .v = odom.v * leg_dir,
            .x = odom.x, .y = odom.y, .theta = odom.theta, .gyro_z = gyro_z,
            .duty_l = duty_l, .duty_r = duty_r,
            .state = (uint8_t)state, .slipping = odom.slipping,
        };
        logger_add(&s);
    }
    return t;
}

static void tick(absolute_time_t now, float dt) {
    int16_t gyro_raw = 0;
    bool imu_ok = mpu6050_read_gyro_z_raw(&gyro_raw);
    bool enc_l_ok = mt6701_update(&enc_left);
    bool enc_r_ok = mt6701_update(&enc_right);
    bool pressed = button_pressed_edge();

    sensor_fail_count = (imu_ok && enc_l_ok && enc_r_ok) ? 0 : sensor_fail_count + 1;
    if (imu_ok) last_gyro_z = mpu6050_gyro_z_radps(gyro_raw);
    float gyro_z = last_gyro_z; // hold the last good reading through a dropped read

    bool running = state == ST_RUN_FWD || state == ST_REVERSE_PAUSE || state == ST_RUN_REV;
    if (running) {
        odom_update(&odom, enc_left.count, enc_right.count, gyro_z, dt);
        if (sensor_fail_count > SENSOR_FAIL_MAX) {
            enter_fault("sensor reads failing");
        } else if (seconds_since(t_run_start, now) > TARGET_TIME_S + RUN_TIMEOUT_EXTRA_S) {
            enter_fault("run timeout");
        } else if (fabsf(odom.theta) > MAX_HEADING_RAD) {
            enter_fault("heading out of range");
        }
    }

    switch (state) {
    case ST_CALIBRATE:
        if (sensor_fail_count > SENSOR_FAIL_MAX) {
            enter_fault("sensor reads failing");
        } else if (imu_ok && mpu6050_cal_add(gyro_raw)) {
            plan_and_arm();
        }
        break;

    case ST_ARMED:
        if (imu_ok) mpu6050_bias_track(gyro_raw);
        if (pressed) {
            start_run(now);
            drive_leg(&fwd_profile, now, dt, gyro_z); // move on this same tick
        }
        break;

    case ST_RUN_FWD: {
        float t = drive_leg(&fwd_profile, now, dt, gyro_z);
        bool stopped = fabsf(odom.v) < V_STOP_MMPS;
        if (t >= fwd_profile.duration && (stopped || t >= fwd_profile.duration + FINISH_TIMEOUT_S)) {
            start_pause(now);
        }
        break;
    }

    case ST_REVERSE_PAUSE:
        motor_brake();
        if (seconds_since(t_pause_start, now) >= T_PAUSE_S) {
            // the reverse plan assumed this start time; using it keeps the schedule honest
            // even if re-planning ran long
            t_leg_start = delayed_by_us(t_pause_start, (uint64_t)(T_PAUSE_S * 1e6f));
            control_reset(&ctrl);
            state = ST_RUN_REV;
        }
        break;

    case ST_RUN_REV: {
        float t = drive_leg(&rev_profile, now, dt, gyro_z);
        bool on_target = fabsf(odom.x - course.target_mm) < FINISH_TOL_MM;
        bool stopped = fabsf(odom.v) < V_STOP_MMPS;
        if (t >= rev_profile.duration &&
            ((on_target && stopped) || t >= rev_profile.duration + FINISH_TIMEOUT_S)) {
            finish_run(now);
        }
        break;
    }

    case ST_FINISHED:
    case ST_FAULT:
        motor_brake();
        if (pressed) {
            if (state == ST_FAULT) printf("FAULT: %s\n", fault_reason);
            logger_dump();
        }
        break;
    }
}

/*
 * LED: fast blink calibrating, slow blink armed (solid if the plan can't hit
 * the target time), on while running, short blip finished, rapid flicker fault.
 */
static void update_led(absolute_time_t now) {
    uint32_t ms = to_ms_since_boot(now);
    bool on;
    switch (state) {
    case ST_CALIBRATE: on = (ms / 125) % 2; break;
    case ST_ARMED:     on = (plan_result == PLAN_OK) ? (ms / 500) % 2 : true; break;
    case ST_FINISHED:  on = (ms % 2000) < 100; break;
    case ST_FAULT:     on = (ms / 50) % 2; break;
    default:           on = true; break;
    }
    gpio_put(LED_PIN, on);
}

int main(void) {
    stdio_init_all();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_init(START_BTN_PIN);
    gpio_set_dir(START_BTN_PIN, GPIO_IN);
    gpio_pull_up(START_BTN_PIN);

    motor_init();
    init_i2c_buses();

    bool imu_ok = mpu6050_init(i2c0, IMU_ADDR);
    bool enc_l_ok = mt6701_init(&enc_left, i2c0, ENC_LEFT_ADDR, ENC_LEFT_INVERT);
    bool enc_r_ok = mt6701_init(&enc_right, i2c1, ENC_RIGHT_ADDR, ENC_RIGHT_INVERT);

#if TEST_MODE
    printf("IMU %s, left encoder %s, right encoder %s\n",
           imu_ok ? "ok" : "MISSING", enc_l_ok ? "ok" : "MISSING", enc_r_ok ? "ok" : "MISSING");
    bench_run(&enc_left, &enc_right);
#endif

    if (!imu_ok) {
        enter_fault("IMU not responding");
    } else if (!enc_l_ok || !enc_r_ok) {
        enter_fault(!enc_l_ok ? "left encoder not responding" : "right encoder not responding");
    } else {
        printf("Calibrating gyro - keep the robot still\n");
        state = ST_CALIBRATE;
    }

    absolute_time_t last = get_absolute_time();
    absolute_time_t next = last;
    for (;;) {
        absolute_time_t now = get_absolute_time();
        float dt = seconds_since(last, now);
        if (dt <= 0.0f || dt > 0.05f) dt = 1.0f / LOOP_HZ;
        last = now;

        tick(now, dt);
        update_led(now);
        tick_count++;

        next = delayed_by_us(next, LOOP_PERIOD_US);
        if (time_reached(next)) {
            overrun_count++;
            next = get_absolute_time();
        } else {
            sleep_until(next);
        }
    }
}
