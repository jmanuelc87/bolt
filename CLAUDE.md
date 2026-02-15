# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Embedded firmware for an STM32F103RCTx (ARM Cortex-M3, 256K Flash / 48K RAM) robotics platform running on a YB-ERF01-V3.0 board. Controls motors, servos, and encoders over UART or CAN bus using FreeRTOS. A host computer (Jetson Orin Nano or PC) sends commands via CANable USB-to-CAN adapter. Written in C++17 (application logic) and C11 (HAL/system code) with no exceptions, no RTTI, and no threadsafe statics.

## Build Commands

```bash
# Configure (choose preset: Debug, RelWithDebInfo, Release, MinSizeRel)
cmake --preset Debug

# Build
cmake --build --preset Debug

# Flash via SWD (requires STM32_Programmer_CLI)
STM32_Programmer_CLI -c port=SWD -w build/Debug/bolt.elf -hardRst
```

Toolchain: `arm-none-eabi-gcc` with Ninja generator, `--specs=nano.specs` (newlib-nano). Build generates `compile_commands.json` for clangd/IDE integration. No unit test framework — testing is done on hardware. Linker script: `STM32F103XX_FLASH.ld` at repo root.

## Architecture

### FreeRTOS Task Pipeline

Five tasks form a command processing pipeline communicating via FreeRTOS queues:

1. **vCommand_Task** (192 words stack) — Receives raw bytes from UART (`USART1`) or CAN bus
2. **vProcess_Task** (384 words stack) — Feeds bytes into `FrameParser` → `FrameDecoder` → `AppVisitor` pipeline
3. **vQuery_Task** (192 words stack) — Sends response frames back over UART or CAN
4. **v2Process_Task** (256 words stack) — Polls `ProcessAsyncTimerPort` registry every 5ms, fires callbacks when counters expire. Drives `EncoderController` and `PIDController` sampling. Implemented in `Bolt/Src/bolt/handle_registry.cpp`
5. **vLed_Task** (128 words stack) — LED heartbeat indicator

FreeRTOS heap budget: 9000 bytes (`configTOTAL_HEAP_SIZE`). `AppQueuesInit()` must be called before tasks run.

### Frame Protocol

Binary protocol: `[SOF=0xAA][TYPE:1][LEN:1][PAYLOAD:LEN][CRC16-CCITT:2][EOF=0x55]`

See @docs/FIRMWARE.md for full spec

### Key Design Patterns

- **Visitor pattern**: `FrameVisitor` base class → `AppVisitor` handles each frame type. Decouples frame definitions (`frames.hpp`) from processing logic.
- **Two-stage parsing**: `FrameParser` (byte-by-byte state machine with CRC validation) emits `RawFrame` structs → `FrameDecoder` maps them to statically-owned `Frame*` objects with per-type length validation.
- **Template Handle Registry**: `HandleRegistry<Controller, HandleType>` maps peripheral handles to C++ controller instances, bridging C interrupt callbacks to OOP controllers. Works with both HAL types (`TIM_HandleTypeDef`) and custom types (`PROC_HandleTypeDef`). CAN ISR dispatch also routes through the registry in `handle_registry.cpp`.
- **Hardware abstraction interfaces** (`Bolt/Inc/bolt/interface/`): `OutputPin`, `SerialPort`, `AsyncSerialPort`, `CanBus`, `PWMTimer`, `CountTimer`, `SpiSyncPort` — all concrete implementations wrap STM32 HAL calls.
- **CAN ISO-TP transport layer**: `CanBusAsyncPort` implements ISO-TP segmentation with single/first/consecutive frames and flow control (std IDs: 0x700 RX, 0x702 TX, 0x701 FC).
- **Compile-time communication switch**: `USE_CANBUS` macro (defined in `CMakeLists.txt`) selects between UART and CAN bus modes.

### Code Layout

- `Bolt/` — Application code (the code you'll modify most)
  - `Inc/bolt/interface/` — Hardware abstraction interfaces and implementations
  - `Inc/bolt/controller/` — Motor, servo, encoder controllers
  - `Inc/bolt/` — Frame definitions, parser state machine, visitor, protocol constants
  - `Src/application.cpp` — FreeRTOS task definitions and main application wiring
  - `Inc/peripherals.hpp` — Global peripheral instances shared across tasks
  - `Inc/queues.hpp` / `Src/queues.cpp` — FreeRTOS queue declarations
- `Core/` — STM32CubeMX-generated HAL configuration (regenerated from `bolt.ioc`)
- `Drivers/` — STM32F1xx HAL drivers and CMSIS (vendor-provided, do not edit)
- `Middlewares/` — FreeRTOS kernel (vendor-provided, do not edit)

### Controller Details

- **MotorController**: 4 motors via PWM on TIM1/TIM8. Signed pulse input (`int16_t`); dead-zone offset of 1600 added internally (actual PWM range 1600–3600). Direction set by complementary CCR registers. Frame protocol uses 1-based motor IDs, code converts to 0-based internally.
- **ServoController (PWM)**: 4 servos via TIM7 software PWM with GPIO pins PC0–PC3 (not hardware PWM channels)
- **ServoController (UART)**: 6 servos via serial protocol on USART3, pulse range 96–4000. Get-angle command retries up to 10× with 2ms delays.
- **EncoderController**: 4 encoders on TIM2/3/4/5, 2464 CPR, 100ms sampling (timer=20 × 5ms tick), low-pass filtered (alpha=0.2)
- **ICM20948Controller**: 9-axis IMU (accel/gyro/mag + temperature) over SPI2 with software NSS (PB12). Uses ICM20948's internal I2C master to read AK09916 magnetometer at 100Hz. Bank-switched register access (4 user banks via register 0x7F). `readAll()` does a 14-byte burst read for accel/gyro/temp, then reads 8 bytes of magnetometer data from the I2C passthrough buffer. Defaults: gyro ±2000 DPS, accel ±16g.
- **PIDController** (WIP): Generic PID controller using `ProcessAsyncTimerPort` for deterministic sampling. Configurable Kp/Ki/Kd gains with output clamping.
- **BeepController**: Buzzer on GPIOC-5. Has `on()`/`off()` methods but not yet wired into `peripherals.hpp`.

### Inter-task Communication

Two FreeRTOS queues (`processQueue`, `queryQueue`) with 8-message buffers each. Message struct: 16-bit size + 38-byte data buffer (`BUFF_SIZE=38`, `MAX_PAYLOAD=32`). Response frames use separate type IDs (PONG=0x01, RPMS=0x02, ANGLE=0x03, IMU=0x04) defined in `Bolt/Inc/definitions.hpp`. Note: RPMS/ANGLE responses encode floats as raw IEEE-754 bytes via `memcpy`, while IMU responses use big-endian packed integers.

### Peripheral Mapping

- **TIM1**: Motors 3-4 (PWM + complementary PWM_N channels)
- **TIM8**: Motors 1-2 (PWM channels)
- **TIM7**: PWM servo software timer (GPIO bit-banging)
- **TIM2/3/4/5**: Encoder counters
- **USART1**: Host communication (when not using CAN)
- **USART3**: Serial servo bus
- **SPI2**: ICM20948 IMU (blocking SPI with software NSS via GPIO)
- **CAN bus**: Accepts all IDs at filter level; software-filters by StdId (0x700 data, 0x701 flow control, 0x702 TX)
- **Debug**: `printf` redirected to USART1 via `__io_putchar` in `Bolt/Inc/debug.h` / `Bolt/Src/debug.c`
- Global peripheral pointers (`gUart1`, `gCanBus`, `gMotorController`, `gImuController`, etc.) initialized in `AppPeripheralsInit()` in `peripherals.hpp`

## Conventions

- All application code lives under `bolt::` namespace with sub-namespaces (`serial`, `can`, `timer`, `controller`, `pin`, `registry`)
- Most implementation is in `.hpp` headers (heavy use of templates and inline functions)
- Inter-task communication uses FreeRTOS queues, never shared globals
- `Core/` files are auto-generated by STM32CubeMX from `bolt.ioc` — edit the `.ioc` file instead of modifying generated code directly
- New source files must be added to `target_sources()` in `CMakeLists.txt`
- Utility functions (`crc16_ccitt`, `u16be`, `build_frame`) are in `Bolt/Inc/utils.h`
- Compile defines: `USE_HAL_DRIVER`, `STM32F103xE`, `DEBUG` (Debug builds only) — set in `cmake/stm32cubemx/CMakeLists.txt`
- Toolchain config in `cmake/gcc-arm-none-eabi.cmake`: `-mcpu=cortex-m3`, C++ extensions ON, `-Wall -Wextra -Wpedantic`
