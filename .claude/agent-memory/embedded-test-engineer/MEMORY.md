# Embedded Test Engineer Agent Memory

## Project Structure
- Tests: `tests/` with separate CMake project (`tests/CMakeLists.txt`)
- Stubs: `tests/stubs/` — shadow real HAL/FreeRTOS headers via first-in-path priority
- Include order: `tests/stubs/` > `Bolt/Inc/` > `Bolt/Inc/bolt/` (set in CMakeLists.txt)
- Test binary: `build/tests/bolt_tests`

## Stub Shadowing Mechanism
When `Bolt/Inc/bolt/visitor.hpp` does `#include "peripherals.hpp"`, compiler searches:
1. Same dir as includer (`Bolt/Inc/bolt/`) — not found
2. `tests/stubs/peripherals.hpp` — FOUND (shadows real one)

Same pattern works for any header not directly in the includer's directory.
Stubs placed at `tests/stubs/<relative-path>` shadow headers included as `"<relative-path>"`.

## Key Global Definitions (defined in test_visitor.cpp)
The following externs declared in stub headers must be defined in exactly one TU:
- `osMessageQueueId_t processQueue, queryQueue` (from `queues.hpp`)
- `osThreadId_t ledTaskHandle` (from `utils.h`, extern "C")
- `TIM_TypeDef gStubTIM1, gStubTIM8` (from `stm32f1xx_hal.h`)
- `GPIO_TypeDef gStubGPIOB, gStubGPIOC, gStubGPIOD` (from `main.h`)
- `SPI_HandleTypeDef hspi2` (from `spi.h`)

## Queue Message Capture (cmsis_os2.h)
Modified `tests/stubs/cmsis_os2.h` to capture the last `osMessageQueuePut` call
in `inline CapturedQueueMessage g_lastQueueMessage`. Tests reset it in `SetUp()`
and inspect via `decodeCapture()` helper in `test_visitor.cpp`.
Message struct layout: `{ uint16_t size; uint8_t data[38]; }` = 40 bytes total.

## AppVisitor Testing Approach
- Include `cmsis_os.h` BEFORE `visitor.hpp` to ensure `vTaskDelay`/`pdMS_TO_TICKS` visible
- Stub `peripherals.hpp` provides inline mock controllers in real namespaces
- Mock classes record last-call args and call counts; no HAL dependencies
- `inline` globals for controller pointers — shared across TUs, reset in TearDown

## Known Production Code Behavior (visitor.hpp)
- `visit(UartServoGetAngleFrame)`: uses INTEGER DIVISION for angle computation:
  `float percent = (pwm - 100) / 3800;` — always 0 or 1, never fractional.
  Tests reflect actual behavior, not intended behavior.
- `visit(UartServoGetAngleFrame)`: loop polls `isReady()` exactly 9 times on timeout
  (i goes from 10, pre-decrements to 9..1, exits when 0, isReady checked 9 times).

## Test Files
- `tests/test_visitor.cpp` — AppVisitor tests (all 11 visit() methods)
- `tests/test_frames.cpp` — Frame struct and visitor dispatch tests
- `tests/test_parser.cpp` — FrameParser state machine + FrameDecoder tests
- `tests/stubs/peripherals.hpp` — Mock controllers (MotorController, PwmServo, UartServo, Encoder, IMU, PIDMotor)
- `tests/stubs/cmsis_os2.h` — Modified: adds message capture capability

## FreeRTOS Stub Notes
- `vTaskDelay` is a no-op in `cmsis_os.h` — safe to use in visitor tests
- `osMessageQueuePut` in `cmsis_os2.h` captures the message (C++17 inline)
- `cmsis_os.h` includes `cmsis_os2.h` — include order matters
