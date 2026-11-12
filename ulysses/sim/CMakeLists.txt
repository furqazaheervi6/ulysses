cmake_minimum_required(VERSION 3.22)

# Standalone firmware for running the Ulysses controls code in the Simantic simulator.
#
# This is not the flight firmware. It builds a small bare-metal test image for a Cortex-M7 that
# runs the controls tests and reports the result over a UART. The STM32H745 that Ulysses flies on
# is not modeled by Simantic, so the image targets an STM32H743, which has the same Cortex-M7 core
# and memory map for the regions used here. See README.md.

# Read by the shared toolchain file at project(), which builds the compiler and linker flags from
# them. The linker script path is relative to this directory.
set(STM32_MCU_FLAGS "-mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard")
set(STM32_LINKER_SCRIPT link.ld)
set(STM32_LINKER_OPTION "-nostartfiles -specs=nosys.specs -Wl,--no-warn-rwx-segments")

project(ulysses_sim C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(ULYSSES_ROOT ${CMAKE_CURRENT_LIST_DIR}/..)

add_executable(controls_sim)

target_sources(controls_sim
    PRIVATE
        controls_test.c
        startup.c
        uart.c
        ${ULYSSES_ROOT}/lib/controls/pid.c
        ${ULYSSES_ROOT}/lib/math/quaternion.c
)

target_include_directories(controls_sim
    PRIVATE
        ${ULYSSES_ROOT}/include
)

target_compile_options(controls_sim
    PRIVATE
        -Wall
        -Wextra
        -Wpedantic
        -Werror
)

set_target_properties(controls_sim PROPERTIES LINK_DEPENDS ${CMAKE_CURRENT_LIST_DIR}/link.ld)

target_link_libraries(controls_sim PRIVATE m)
