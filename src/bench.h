#ifndef BENCH_H
#define BENCH_H

#include "mt6701.h"

// TEST_MODE bench checks (wheels off the ground). Never returns.
void bench_run(mt6701_t *enc_l, mt6701_t *enc_r);

#endif
