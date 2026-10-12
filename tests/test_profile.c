// Host-side test for the motion planner: make -C tests
#include <math.h>
#include <stdio.h>
#include "profile.h"

static profile_t fwd, rev;
static int failures = 0;

#define CHECK(cond, ...) do { if (!(cond)) { failures++; printf("  FAIL: " __VA_ARGS__); printf("\n"); } } while (0)

// sample a profile every 1 ms and check it is physically sensible
static void check_profile(profile_t *p, const char *name) {
    float last_s = 0.0f, last_v = 0.0f;
    for (float t = 0.0f; t <= p->duration + 0.01f; t += 0.001f) {
        float s, v;
        profile_sample(p, t, &s, &v);
        CHECK(s >= last_s - 1e-3f, "%s: progress went backwards at t=%.3f", name, t);
        CHECK(v >= -1e-3f && v <= V_MAX_MMPS + 1.0f, "%s: speed %.1f out of range", name, v);
        if (t > 0.0f) {
            float a = (v - last_v) / 0.001f;
            CHECK(fabsf(a) <= A_MAX_MMPS2 * 1.02f, "%s: accel %.0f at t=%.3f", name, a, t);
        }
        last_s = s;
        last_v = v;
    }
    float s, v;
    profile_sample(p, p->duration + 1.0f, &s, &v);
    CHECK(fabsf(s - p->length) < 0.01f && v == 0.0f, "%s: does not end at rest at its length", name);
}

// speed at a given progress, by scanning time (only used for zone checks)
static float speed_at(profile_t *p, float s_query) {
    for (float t = 0.0f; t <= p->duration; t += 0.001f) {
        float s, v;
        profile_sample(p, t, &s, &v);
        if (s >= s_query) return v;
    }
    return 0.0f;
}

static const char *result_name(plan_result_t r) {
    switch (r) {
        case PLAN_OK: return "ok";
        case PLAN_TOO_FAST: return "TOO_FAST";
        case PLAN_TOO_SLOW: return "TOO_SLOW";
    }
    return "?";
}

int main(void) {
    const float targets[] = {5000.0f, 7000.0f, 9000.0f};
    const float lines[] = {1000.0f, 2000.0f, 3000.0f};
    const float times[] = {10.0f, 12.5f, 15.0f, 20.0f};
    const float bottles[] = {2000.0f, 3000.0f};

    printf("%-7s %-7s %-6s %-6s %-9s %-8s %-8s %s\n",
           "bottle", "target", "line", "time", "result", "cruise", "moving", "budget");
    for (int d = 0; d < 2; d++) {
        for (int a = 0; a < 3; a++) {
            for (int b = 0; b < 3; b++) {
                for (int c = 0; c < 4; c++) {
                    course_t course = {targets[a], lines[b], bottles[d]};
                    float budget = times[c] - T_PAUSE_S - T_END_MARGIN_S;
                    plan_result_t r = plan_run(&fwd, &rev, &course, budget);
                    float moving = fwd.duration + rev.duration;

                    printf("%-7.0f %-7.0f %-6.0f %-6.1f %-9s %-8.0f %-8.3f %.3f\n",
                           bottles[d], targets[a], lines[b], times[c], result_name(r),
                           fwd.cruise, moving, budget);

                    float fwd_len = plan_forward_stop_mm(&course);
                    CHECK(fabsf(fwd.length - fwd_len) < 0.01f, "forward length");
                    CHECK(fabsf(rev.length - (fwd_len - targets[a])) < 0.01f, "reverse length");
                    if (r == PLAN_OK) {
                        CHECK(fabsf(moving - budget) < 0.01f, "moving time %.3f != budget %.3f", moving, budget);
                    } else if (r == PLAN_TOO_FAST) {
                        CHECK(moving > budget, "TOO_FAST but fits");
                    }

                    float contact = bottles[d] - MP_TO_BUMPER_MM;
                    CHECK(speed_at(&fwd, contact) <= V_CONTACT_MMPS + 1.0f,
                          "hits bottle at %.0f mm/s", speed_at(&fwd, contact));
                    CHECK(speed_at(&rev, rev.length - CREEP_DIST_MM * 0.5f) <= V_CREEP_MMPS + 1.0f,
                          "final approach too fast");

                    check_profile(&fwd, "fwd");
                    check_profile(&rev, "rev");
                }
            }
        }
    }

    // re-planning the reverse leg with less time left should speed it up
    profile_t slow, fast;
    plan_reverse(&slow, 2000.0f, 8.0f);
    plan_reverse(&fast, 2000.0f, 5.0f);
    CHECK(fast.cruise > slow.cruise, "replan with less time did not speed up");
    CHECK(fabsf(fast.duration - 5.0f) < 0.01f, "replan duration %.3f", fast.duration);

    printf("\n%s (%d failures)\n", failures ? "FAILED" : "PASSED", failures);
    return failures ? 1 : 0;
}
