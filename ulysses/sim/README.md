# Simulation test image

A small bare-metal test image that runs the Ulysses controls code (PID and quaternion helpers) in
the [Simantic](https://simantic.com/docs) simulator. It is not flight firmware. It prints one
`ULYSSES-SIM: <test> PASS` line per test over a UART, then `ULYSSES-SIM: ALL PASS`, and CI asserts on
that output.

The STM32H745 that Ulysses flies on is not modeled by Simantic. The image therefore targets an
STM32H743, which has the same Cortex-M7 core and the same flash and AXI SRAM addresses used here.
It does not exercise the CM4 core, the HAL, FreeRTOS or any real peripheral.

## Building

```sh
cd ulysses/sim
cmake --preset Debug
cmake --build --preset Debug
```

## Running

```sh
sim STM32H743ZI run --elf build/Debug/controls_sim.elf --ascii --timeout 2 \
    --expect "ULYSSES-SIM: ALL PASS" --expect-absent "FAIL"
```

CI runs this in the `Simulate controls (Simantic)` job of `.github/workflows/ulysses-firmware.yaml`.
It needs a repository secret named `TEST3`, an `smtc_` API key created under Account, API
Settings on simantic.com. Without the secret, the job skips itself.

## Adding tests

Add a `test_*` function to `controls_test.c`, call `report()` with a name and result, and call it
from `main()`. The flight controller is not covered yet, since `flight_controller.h` includes
`ulysses/lib/eskf/eskf.h`, which this branch does not have.
