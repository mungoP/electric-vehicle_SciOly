cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(firmware C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_link_options(--specs=nosys.specs --specs=nano.specs)

pico_sdk_init()

set(FIRMWARE_SOURCES
    main.c
    src/bench.c
    src/control.c
    src/logger.c
    src/motor.c
    src/mpu6050.c
    src/mt6701.c
    src/odometry.c
    src/profile.c
)

add_executable(firmware ${FIRMWARE_SOURCES})

# warnings for our code only (SDK sources are compiled into the same target)
set_source_files_properties(${FIRMWARE_SOURCES} PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra")

target_link_libraries(firmware pico_stdlib hardware_i2c hardware_pwm)
target_include_directories(firmware PRIVATE include src)

option(EV_TEST_MODE "Build the bench test program instead of the run program" OFF)
if (EV_TEST_MODE)
    target_compile_definitions(firmware PRIVATE TEST_MODE=1)
endif()

pico_enable_stdio_usb(firmware 1)
pico_enable_stdio_uart(firmware 0)

pico_add_extra_outputs(firmware)
