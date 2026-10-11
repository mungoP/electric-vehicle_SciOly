cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(firmware C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17) 
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_link_options(--specs=nosys.specs --specs=nano.specs)

pico_sdk_init()

add_executable(firmware main.c)

target_link_libraries(firmware pico_stdlib hardware_i2c)
target_include_directories(firmware PRIVATE include)

pico_enable_stdio_usb(firmware 1) 
pico_enable_stdio_uart(firmware 0) 

pico_add_extra_outputs(firmware) 


