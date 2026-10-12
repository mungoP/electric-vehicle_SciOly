# electric-vehicle_SciOly

Source code for a Raspberry pico based robot for the Science Olympiad event "Electric Vehicle."

## Competition setup
1. Edit the **COMPETITION SETTINGS** block at the top of `include/config.h`
   (`TARGET_DISTANCE_MM`, `TARGET_TIME_S`, `BOTTLE_LINE_FROM_TARGET_MM`, `BOTTLE_START_MM`).
2. `cmake -S . -B build && cmake --build build`, then flash `build/firmware.uf2`.
3. Power on with the robot still. LED: fast blink = calibrating gyro, slow blink = armed,
   solid = armed but the target time is faster than `V_MAX_MMPS` allows.
4. Press START. After the run, connect USB and press START again to print the CSV log.

## Bench tests (first time, or after hardware changes)
`cmake -S . -B build-bench -DEV_TEST_MODE=ON && cmake --build build-bench`, flash, open the
USB serial monitor, and step through the tests with START (wheels off the ground). They check
the motor/encoder/gyro direction flags and measure `KV_DUTY_PER_MMPS`, `KS_DUTY` and top speed.

## Planner test (on your computer)
`make -C tests` checks the motion planner over every course/time combination and prints
which ones the robot can't finish in time at `V_MAX_MMPS`.
