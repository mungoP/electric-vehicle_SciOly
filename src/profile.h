#ifndef PROFILE_H
#define PROFILE_H

#include <stdint.h>
#include "config.h"

// speed vs time reference for one leg (forward or reverse), starts and ends at rest
typedef struct {
    int n;          // number of grid points
    float ds;       // grid spacing (mm)
    float length;   // leg length (mm)
    float duration; // leg time (s)
    float cruise;   // cruise speed chosen by the planner (mm/s)
    int cursor;     // sampling search hint
    float t[PROFILE_MAX_PTS]; // time at each grid point
    float v[PROFILE_MAX_PTS]; // speed at each grid point
} profile_t;

typedef enum {
    PLAN_OK,
    PLAN_TOO_FAST,  // budget too short even at V_MAX, will finish late
    PLAN_TOO_SLOW,  // budget too long even at V_MIN_CRUISE, will finish early
} plan_result_t;

// the course as posted after impound (all mm, measured as in config.h)
typedef struct {
    float target_mm;            // start point -> target point
    float line_from_target_mm;  // target point -> bottle line
    float bottle_start_mm;      // start point -> bottle
} course_t;

// course built from the config.h competition settings
course_t course_from_config(void);

// MP position (from the start point) where the forward leg stops
float plan_forward_stop_mm(const course_t *course);

// plan both legs with one shared cruise speed so their total time equals budget_s
plan_result_t plan_run(profile_t *fwd, profile_t *rev, const course_t *course, float budget_s);

// re-plan the reverse leg only (from the actual stop position and remaining time)
plan_result_t plan_reverse(profile_t *rev, float length_mm, float budget_s);

// reference progress (mm) and speed (mm/s) at time t (s) since the leg started
void profile_sample(profile_t *p, float t, float *s_ref, float *v_ref);

#endif
