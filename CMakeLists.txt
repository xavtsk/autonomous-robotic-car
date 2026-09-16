cmake_minimum_required(VERSION 3.13...3.27)

# Import the SDK before project() so it can select the ARM toolchain.
include(pico_sdk_import.cmake)

project(autonomous_robotic_car C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

pico_sdk_init()

# Only the development smoke test is built at this stage.
add_executable(pico_hello src/main.c)
target_link_libraries(pico_hello PRIVATE pico_stdlib)

pico_enable_stdio_usb(pico_hello 1)
pico_enable_stdio_uart(pico_hello 0)
pico_add_extra_outputs(pico_hello)
