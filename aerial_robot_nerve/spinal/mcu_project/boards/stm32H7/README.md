# STM32H7 Spinal - Build Guide

This document describes how to build the stm32H7 spinal firmware with **STM32CubeIDE** and how its serial links are configured.

## Table of Contents

- [Hardware](#hardware)
- [1. Build Firmware with CubeIDE](#1-build-firmware-with-cubeide)
- [2. Serial Communication](#2-serial-communication)
- [Troubleshooting](#troubleshooting)

---

## Hardware

| Item | Description |
|------|-------------|
| MCU | STM32H743VITx (Cortex-M7) |
| RTOS | FreeRTOS |
| Communication | UART (rosserial) / Ethernet (LwIP) / FDCAN |

---

## 1. Build Firmware with CubeIDE

Tested with STM32CubeIDE 1.18.0, which includes the ARM GCC toolchain. Download it from the [ST website](https://www.st.com/en/development-tools/stm32cubeide.html).

### 1.1 Import the Project

1. Open STM32CubeIDE
2. `File` → `Import` → `General` → `Existing Projects into Workspace`
3. Select directory: `<this-repo>/boards/stm32H7/STM32CubeIDE`
4. Keep **Copy projects into workspace** unchecked, check the `spinal` project and click `Finish`

### 1.2 Build

1. Right-click the `spinal` project in Project Explorer
2. Select `Build Project` (or press `Ctrl+B`)
3. Build artifacts are located in `STM32CubeIDE/Debug/`: `spinal.elf`, `spinal.bin`, `spinal.map`

### 1.3 Flash

Connect the ST-Link and use `Run` → `Debug As` → `STM32 C/C++ Application` in CubeIDE, or flash the binary directly:
```bash
st-flash write spinal.bin 0x08000000
```

> After changing any message in `spinal/msg`, rebuild `spinal` with catkin so that `ros_lib` is regenerated, then rebuild and flash the firmware.

---

## 2. Serial Communication

### Change serial bitrate
The rosserial link (`huart1`) runs at `2000000` (it was `921600`, which is not enough once the servo states are fed back at `200Hz`). The bitrate is set in `main.c`, and the robot's `sensors.launch.xml` must pass the same value to `bridge.launch` (the default of `bridge.launch` stays `921600` for the stm32F7 boards).
```c++
huart1.Init.BaudRate = 2000000; // serial bitrate configuration
huart1.Init.OverSampling = UART_OVERSAMPLING_8;
```
```xml
<include file="$(find spinal)/launch/bridge.launch" >
  <arg name="mode" value="serial" />
  <arg name="serial_port" value="/dev/flight_controller" />
  <arg name="serial_baud" value="2000000" />
</include>
```

### Change servo feedback rate
In the `main.c`, define servo uart bitrate, also `huart3` is used for servo communication.
```c++
osTimerStart(ServoTimerHandle, 1);  // Servo Task period
huart3.Init.BaudRate = 1000000; // servo serial bitrate configuration
```
In the `dynamixel_serial.h` define the servo feedback rate. Make sure `GET_POS_OFFSET != SET_POS_OFFSET` and `GET_POS_OFFSET < GET_POS_DU`.
```c++
#define SET_POS_DU 5 //[msec], 5ms => 200Hz
#define SET_POS_OFFSET 0 // offset from SET_POS
#define GET_POS_DU 5 //[msec], 5ms => 200Hz
#define GET_POS_OFFSET 2 //offset from GET_POS
```
The [`FastSyncRead`](https://emanual.robotis.com/docs/en/dxl/protocol2/#fast-sync-read-0x8a) function is defined in `dynamixel_serial.h` by
```c++
#ifndef DYNAMIXEL_USE_FAST_SYNC_READ
#define DYNAMIXEL_USE_FAST_SYNC_READ 1 // 0: default SyncRead, 1: FastSyncRead
#endif
```
This function saves bandwidth and speeds up UART communication.

### ESC telemetry
`huart2` receives the KISS telemetry of the ESCs at `460800` (aligned with AM32). The request schedule is set in `dshot.h`:
```c++
#define ESC_TELEM_REQUEST_DIVIDER 1
#define ESC_TELEM_SLOT_COUNT 5  // slot 0: idle, slots 1-4: ESC 1-4
```
The per-ESC telemetry rate is `1kHz / ESC_TELEM_REQUEST_DIVIDER / ESC_TELEM_SLOT_COUNT`, i.e., `200Hz` by default.

---


## Troubleshooting

### Q1: STM32CubeIDE hangs at the splash/progress bar after selecting a workspace

**Symptoms**:
- The progress bar reaches the last part and then stops.
- The terminal repeatedly prints:
  ```text
  1 : Invalid condition id : UX_CORESTACK_Condition cause : null
  ```

**Likely cause**: This is usually a CubeIDE/CubeMX workspace or plug-in cache issue, not a firmware source-code issue. The `UX_CORESTACK_Condition` message comes from CubeMX parsing third-party PDSC packs such as Azure RTOS/USBX. It can appear together with stale workspace metadata under `.metadata` or incompatible third-party pack cache under `~/.stm32cubemx/thirdparties`.

**Recommended recovery**:
```bash
# Close STM32CubeIDE first.

# Prefer a clean workspace that is separate from the STM32Cube firmware repository.
mkdir -p ~/STM32CubeIDE/workspace_1.18.0
stm32cubeide \
  -clean -clearPersistedState \
  -data ~/STM32CubeIDE/workspace_1.18.0
```

If `stm32cubeide` is not in your `PATH`, use the full path instead:
```bash
~/st/stm32cubeide_1.18.0/stm32cubeide_wayland \
  -clean -clearPersistedState \
  -data ~/STM32CubeIDE/workspace_1.18.0
```

Then import the project again:
1. `File` -> `Import` -> `General` -> `Existing Projects into Workspace`
2. Select directory: `<this-repo>/boards/stm32H7/STM32CubeIDE`
3. Keep **Copy projects into workspace** unchecked

If it still hangs, back up and regenerate only the metadata/cache, then restart CubeIDE:
```bash
# Replace OLD_WORKSPACE with the workspace path you selected in CubeIDE.
mv OLD_WORKSPACE/.metadata OLD_WORKSPACE/.metadata.bak.$(date +%Y%m%d-%H%M%S)

# If CubeMX third-party pack loading is still the last log entry:
mv ~/.stm32cubemx/thirdparties ~/.stm32cubemx/thirdparties.bak.$(date +%Y%m%d-%H%M%S)
```

These commands do not delete project sources; they only make CubeIDE recreate workspace metadata and CubeMX third-party pack indexes.

---
