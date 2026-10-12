#include <stdio.h>
#include "config.h"
#include "logger.h"

static log_sample_t samples[LOG_MAX_SAMPLES];
static int n_samples;

void logger_clear(void) {
    n_samples = 0;
}

void logger_add(const log_sample_t *s) {
    if (n_samples < LOG_MAX_SAMPLES) {
        samples[n_samples++] = *s;
    }
}

void logger_dump(void) {
    printf("\n--- LOG BEGIN (%d samples) ---\n", n_samples);
    printf("t,state,p_ref,p,v_ref,v,x,y,theta,gyro_z,duty_l,duty_r,slip\n");
    for (int i = 0; i < n_samples; i++) {
        const log_sample_t *s = &samples[i];
        printf("%.3f,%u,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.4f,%.4f,%.3f,%.3f,%u\n",
               s->t, s->state, s->p_ref, s->p, s->v_ref, s->v, s->x, s->y,
               s->theta, s->gyro_z, s->duty_l, s->duty_r, s->slipping);
    }
    printf("--- LOG END ---\n");
}
