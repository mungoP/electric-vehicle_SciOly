#ifndef CONFIG_H
#define CONFIG_H

/*
 * ============================================================
 *  COMPETITION SETTINGS - edit these, rebuild, and flash
 * ============================================================
 * All distances are in mm, all times in seconds.
 */
#define TARGET_DISTANCE_MM          7000.0f // start point -> target point (5000-9000)
#define TARGET_TIME_S               15.0f   // target run time (10.0-20.0, 0.5 s steps)
#define BOTTLE_LINE_FROM_TARGET_MM  2000.0f // target point -> bottle line (1000-3000)
#define BOTTLE_START_MM             2000.0f // start point -> bottle (2000 regionals, 3000 states)
#define BOTTLE_MARGIN_MM            30.0f   // how far past the bottle line to push the bottle

// set to 1 to build the bench test program instead of the run program
// (or build with: cmake -B build-bench -DEV_TEST_MODE=ON)
#ifndef TEST_MODE
#define TEST_MODE 0
#endif

/*
 * ============================================================
 *  ROBOT GEOMETRY + CALIBRATION
 * ============================================================
 */
#define WHEEL_DIAMETER_MM   60.0f
#define TRACK_WIDTH_MM      147.0f  // distance between wheel centers
#define MP_TO_BUMPER_MM     261.0f  // rear dowel (measurement point) -> front bumper

// effective wheel size multipliers, tune from measured straight runs
// (measured distance / distance the robot reported)
#define WHEEL_SCALE_L       1.000f
#define WHEEL_SCALE_R       1.000f

// direction flags - verify all of these with TEST_MODE before the first run
#define LEFT_MOTOR_INVERT   0
#define RIGHT_MOTOR_INVERT  0
#define ENC_LEFT_INVERT     0
#define ENC_RIGHT_INVERT    0
#define GYRO_Z_INVERT       0       // turning left (CCW from above) must read positive

/*
 * ============================================================
 *  MOTION PROFILE
 * ============================================================
 */
#define V_MAX_MMPS              1400.0f // fastest cruise speed the planner may use
#define V_MIN_CRUISE_MMPS       80.0f   // slowest cruise speed the planner may use
#define A_MAX_MMPS2             1000.0f // accel/decel limit (keep below bottle friction decel)
#define V_CONTACT_MMPS          250.0f  // speed when the bumper meets the bottle
#define CONTACT_SLOW_BEFORE_MM  120.0f  // be at contact speed this far before the bottle
#define CONTACT_SLOW_AFTER_MM   40.0f   // stay at contact speed this far after touching it
#define V_PUSH_MAX_MMPS         1200.0f // speed cap while pushing the bottle
#define V_CREEP_MMPS            80.0f   // final approach speed into the target point
#define CREEP_DIST_MM           60.0f   // length of the final approach
#define T_PAUSE_S               0.25f   // stopped time between forward and reverse
#define T_END_MARGIN_S          0.15f   // planned finish this much before target time
#define PROFILE_DS_MM           5.0f    // planner resolution
#define PROFILE_MAX_PTS         2600

/*
 * ============================================================
 *  CONTROL GAINS (starting points - tune on the floor)
 * ============================================================
 */
#define LOOP_HZ             200
#define LOOP_PERIOD_US      (1000000 / LOOP_HZ)

// position (schedule) loop
#define KP_POS              4.0f    // mm/s of correction per mm behind schedule
#define V_CORR_MAX_MMPS     250.0f  // limit on the position correction

// per-wheel velocity loop: duty = KV*v + KS*sign(v) + PI(error)
#define KV_DUTY_PER_MMPS    (1.0f / 1500.0f) // from TEST_MODE duty sweep
#define KS_DUTY             0.06f   // duty needed to overcome static friction
#define V_FF_DEADBAND_MMPS  2.0f    // no KS kick below this commanded speed
#define KP_VEL              0.0008f
#define KI_VEL              0.004f
#define VEL_INTEG_MAX       0.3f

// heading PD: dv (mm/s, right minus left) = KP*(err) - KD*(gyro rate)
#define KP_HEAD             800.0f  // mm/s per rad
#define KD_HEAD             60.0f   // mm/s per rad/s
#define HEAD_DV_FRAC        0.3f    // max differential as fraction of speed...
#define HEAD_DV_MIN_MMPS    30.0f   // ...but always allow at least this much

// cross-track correction (steer back toward the start line's axis), 0 = off
#define K_CROSS_RAD_PER_MM  0.0f
#define CROSS_MAX_RAD       0.05f

/*
 * ============================================================
 *  SENSORS
 * ============================================================
 */
#define I2C_BAUD                400000
#define I2C_TIMEOUT_US          2000
#define GYRO_LSB_PER_DPS        65.5f   // +-500 dps range
#define CAL_SAMPLES             400     // 2 s at 200 Hz
#define CAL_STILL_STD_LSB       8.0f    // max gyro noise allowed during calibration
#define BIAS_TRACK_ALPHA        0.002f  // slow bias refinement while armed
#define BIAS_TRACK_STILL_LSB    25.0f   // only refine when this close to the bias

#define VEL_FILTER_TAU_S        0.02f   // wheel speed low-pass
#define HEADING_FUSION_TAU_S    10.0f   // encoder heading correction time constant (0 = gyro only)
#define SLIP_THRESH_RADPS       0.35f   // gyro vs encoder yaw rate mismatch = slip
#define SLIP_HOLD_S             0.1f

/*
 * ============================================================
 *  FINISH + SAFETY
 * ============================================================
 */
#define FINISH_TOL_MM           2.0f
#define V_STOP_MMPS             15.0f
#define FINISH_TIMEOUT_S        0.75f   // give up converging this long after the profile ends
#define RUN_TIMEOUT_EXTRA_S     8.0f    // fault if the run takes this much longer than target
#define MAX_HEADING_RAD         0.52f   // ~30 deg
#define SENSOR_FAIL_MAX         10      // consecutive failed reads before faulting

#define LOG_DIVIDER             2       // log every 2nd tick (100 Hz)
#define LOG_MAX_SAMPLES         2400

// catch typos in the competition settings at compile time
_Static_assert(TARGET_DISTANCE_MM >= 5000.0f && TARGET_DISTANCE_MM <= 9000.0f,
               "TARGET_DISTANCE_MM must be 5000-9000");
_Static_assert(TARGET_TIME_S >= 10.0f && TARGET_TIME_S <= 20.0f,
               "TARGET_TIME_S must be 10.0-20.0");
_Static_assert(BOTTLE_LINE_FROM_TARGET_MM >= 1000.0f && BOTTLE_LINE_FROM_TARGET_MM <= 3000.0f,
               "BOTTLE_LINE_FROM_TARGET_MM must be 1000-3000");
_Static_assert(BOTTLE_START_MM > MP_TO_BUMPER_MM && BOTTLE_START_MM < TARGET_DISTANCE_MM,
               "BOTTLE_START_MM must be between the bumper and the target");

#endif
