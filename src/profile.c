#include <math.h>
#include <stddef.h>
#include "profile.h"

#define BISECT_ITERS 24

// a stretch of the leg with a lower speed limit
typedef struct {
    float start;
    float end;
    float vmax;
} zone_t;

course_t course_from_config(void) {
    course_t c = {TARGET_DISTANCE_MM, BOTTLE_LINE_FROM_TARGET_MM, BOTTLE_START_MM};
    return c;
}

float plan_forward_stop_mm(const course_t *course) {
    // bumper (and the bottle's near edge) ends up BOTTLE_MARGIN_MM past the line
    return course->target_mm + course->line_from_target_mm + BOTTLE_MARGIN_MM - MP_TO_BUMPER_MM;
}

static float speed_cap(float s, float v_cruise, const zone_t *zones, int n_zones) {
    float v = v_cruise;
    for (int i = 0; i < n_zones; i++) {
        if (s >= zones[i].start && s <= zones[i].end && zones[i].vmax < v) {
            v = zones[i].vmax;
        }
    }
    return v;
}

/*
 * Builds the fastest profile that respects the speed caps and A_MAX_MMPS2.
 * Works on a distance grid: cap every point, then sweep forward (acceleration
 * limit) and backward (deceleration limit). Constant acceleration between grid
 * points makes v^2 linear in s, so each step takes exactly 2*ds/(v0+v1).
 */
static void build(profile_t *p, float length, float v_cruise, const zone_t *zones, int n_zones) {
    p->length = length;
    p->cruise = v_cruise;
    p->cursor = 0;

    if (length < 1.0f) {
        p->n = 1;
        p->ds = 0.0f;
        p->t[0] = 0.0f;
        p->v[0] = 0.0f;
        p->duration = 0.0f;
        return;
    }

    int n = (int)ceilf(length / PROFILE_DS_MM) + 1;
    if (n > PROFILE_MAX_PTS) n = PROFILE_MAX_PTS;
    if (n < 2) n = 2;
    float ds = length / (float)(n - 1);
    p->n = n;
    p->ds = ds;

    for (int i = 0; i < n; i++) {
        p->v[i] = speed_cap((float)i * ds, v_cruise, zones, n_zones);
    }
    p->v[0] = 0.0f;
    p->v[n - 1] = 0.0f;

    float two_a_ds = 2.0f * A_MAX_MMPS2 * ds;
    for (int i = 1; i < n; i++) {
        float lim = sqrtf(p->v[i - 1] * p->v[i - 1] + two_a_ds);
        if (p->v[i] > lim) p->v[i] = lim;
    }
    for (int i = n - 2; i >= 0; i--) {
        float lim = sqrtf(p->v[i + 1] * p->v[i + 1] + two_a_ds);
        if (p->v[i] > lim) p->v[i] = lim;
    }

    p->t[0] = 0.0f;
    for (int i = 1; i < n; i++) {
        float v_avg = 0.5f * (p->v[i - 1] + p->v[i]);
        if (v_avg < 1e-3f) v_avg = 1e-3f;
        p->t[i] = p->t[i - 1] + ds / v_avg;
    }
    p->duration = p->t[n - 1];
}

// contact = MP position when the bumper reaches the bottle
static void build_forward(profile_t *p, float length, float contact, float v_cruise) {
    zone_t zones[2] = {
        {contact - CONTACT_SLOW_BEFORE_MM, contact + CONTACT_SLOW_AFTER_MM, V_CONTACT_MMPS},
        {contact, length, V_PUSH_MAX_MMPS},
    };
    build(p, length, v_cruise, zones, 2);
}

static void build_reverse(profile_t *p, float length, float v_cruise) {
    zone_t creep = {length - CREEP_DIST_MM, length, V_CREEP_MMPS};
    build(p, length, v_cruise, &creep, 1);
}

typedef struct {
    profile_t *fwd;     // NULL when planning the reverse leg only
    float fwd_len;
    float contact;
    profile_t *rev;
    float rev_len;
} legs_t;

// total time of the legs at a cruise speed (leaves them built)
static float total_time(const legs_t *l, float v) {
    float t = 0.0f;
    if (l->fwd != NULL) {
        build_forward(l->fwd, l->fwd_len, l->contact, v);
        t += l->fwd->duration;
    }
    build_reverse(l->rev, l->rev_len, v);
    return t + l->rev->duration;
}

/*
 * Time decreases as cruise speed increases, so bisect for the cruise speed
 * whose total time matches the budget. Leaves the profiles built.
 */
static plan_result_t solve(const legs_t *l, float budget_s) {
    float lo = V_MIN_CRUISE_MMPS;
    float hi = V_MAX_MMPS;

    if (total_time(l, lo) < budget_s) {
        return PLAN_TOO_SLOW;
    }
    if (total_time(l, hi) > budget_s) {
        return PLAN_TOO_FAST;
    }
    for (int i = 0; i < BISECT_ITERS; i++) {
        float mid = 0.5f * (lo + hi);
        if (total_time(l, mid) > budget_s) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    total_time(l, hi);
    return PLAN_OK;
}

plan_result_t plan_run(profile_t *fwd, profile_t *rev, const course_t *course, float budget_s) {
    float fwd_len = plan_forward_stop_mm(course);
    legs_t l = {
        .fwd = fwd,
        .fwd_len = fwd_len,
        .contact = course->bottle_start_mm - MP_TO_BUMPER_MM,
        .rev = rev,
        .rev_len = fwd_len - course->target_mm,
    };
    return solve(&l, budget_s);
}

plan_result_t plan_reverse(profile_t *rev, float length_mm, float budget_s) {
    legs_t l = {.fwd = NULL, .rev = rev, .rev_len = length_mm};
    return solve(&l, budget_s);
}

void profile_sample(profile_t *p, float t, float *s_ref, float *v_ref) {
    if (p->n < 2 || t <= 0.0f) {
        *s_ref = 0.0f;
        *v_ref = 0.0f;
        return;
    }
    if (t >= p->duration) {
        *s_ref = p->length;
        *v_ref = 0.0f;
        return;
    }

    if (p->cursor >= p->n - 1 || t < p->t[p->cursor]) p->cursor = 0;
    while (p->cursor < p->n - 2 && t >= p->t[p->cursor + 1]) p->cursor++;

    // constant acceleration inside the grid step
    int i = p->cursor;
    float step_t = p->t[i + 1] - p->t[i];
    float a = (step_t > 0.0f) ? (p->v[i + 1] - p->v[i]) / step_t : 0.0f;
    float tau = t - p->t[i];
    *v_ref = p->v[i] + a * tau;
    *s_ref = (float)i * p->ds + p->v[i] * tau + 0.5f * a * tau * tau;
}
