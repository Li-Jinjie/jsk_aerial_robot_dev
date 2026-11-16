# STM32H7 Spinal - Build & Debug Guide

This document describes how to build firmware using **STM32CubeIDE** and debug it using **VS Code + OpenOCD + Cortex-Debug**.

## Table of Contents

- [Hardware Setup](#hardware-setup)
- [Software Dependencies](#software-dependencies)
- [1. Install STM32CubeIDE](#1-install-stm32cubeide)
- [2. Install OpenOCD](#2-install-openocd)
- [3. Install GDB Dependencies (libncurses5 / libtinfo5)](#3-install-gdb-dependencies-libncurses5--libtinfo5)
- [4. Install VS Code Extensions](#4-install-vs-code-extensions)
- [5. Build Firmware with CubeIDE](#5-build-firmware-with-cubeide)
- [6. Debug with VS Code](#6-debug-with-vs-code)
- [7. Debug Features](#7-debug-features)
- [8. Serial Communication](#8-serial-communication)
- [Troubleshooting](#troubleshooting)

---

## Hardware Setup

| Item | Description |
|------|-------------|
| MCU | STM32H743VITx (Cortex-M7) |
| Debugger | ST-Link V2 |
| Wiring | SWCLK, SWDIO, GND, 5V (**no NRST**) |
| RTOS | FreeRTOS |
| Communication | UART (rosserial) / Ethernet (LwIP) / FDCAN |

> ⚠️ The NRST (hardware reset) line is **not connected** in this project. Software reset via the Cortex-M AIRCR register is used instead.

---

## Software Dependencies

| Software | Tested Version | Purpose |
|----------|---------------|---------|
| Ubuntu | 24.04 LTS | Host OS |
| STM32CubeIDE | 1.18.0 | Build firmware (includes ARM GCC toolchain) |
| OpenOCD | 0.12.0 | Debug server, connects to ST-Link |
| VS Code | Latest | Code editor + debugger |
| Cortex-Debug extension | 1.12.1+ | ARM Cortex-M debug support for VS Code |
| libncurses5 / libtinfo5 | 6.1 | Runtime dependency for ARM GDB |

---

## 1. Install STM32CubeIDE

Download STM32CubeIDE for Linux from the [ST website](https://www.st.com/en/development-tools/stm32cubeide.html) and install it.

Default installation path: `~/st/stm32cubeide_1.18.0/`

After installation, the ARM GCC toolchain is located at:
```
~/st/stm32cubeide_1.18.0/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3.rel1.linux64_1.0.0.202410170706/tools/bin/
```

---

## 2. Install OpenOCD

```bash
sudo apt update
sudo apt install openocd
```

Verify installation:
```bash
openocd --version
# Expected: Open On-Chip Debugger 0.12.0
```

This project includes a pre-configured OpenOCD config file:
- `STM32CubeIDE/openocd_h743_stlink_v2.cfg` — configured for ST-Link V2 without NRST connection

---

## 3. Install GDB Dependencies (libncurses5 / libtinfo5)

The `arm-none-eabi-gdb` bundled with STM32CubeIDE depends on `libncurses.so.5` and `libtinfo.so.5`. Ubuntu 24.04 ships with v6 by default and does not include these libraries, so they must be installed manually.

### Method: Download deb packages and extract manually

```bash
# Create target directory
mkdir -p ~/drivers/libncurses5

# Download deb packages (Ubuntu 18.04 version, compatible)
cd ~/drivers/
wget http://archive.ubuntu.com/ubuntu/pool/universe/n/ncurses/libncurses5_6.1-1ubuntu1.18.04.1_amd64.deb
wget http://archive.ubuntu.com/ubuntu/pool/universe/n/ncurses/libtinfo5_6.1-1ubuntu1.18.04.1_amd64.deb

# Extract to a custom path (avoids polluting system directories)
dpkg-deb -x libncurses5_6.1-1ubuntu1.18.04.1_amd64.deb ~/drivers/libncurses5/
dpkg-deb -x libtinfo5_6.1-1ubuntu1.18.04.1_amd64.deb ~/drivers/libncurses5/
```

After extraction, the library files are located at:
```
~/drivers/libncurses5/lib/x86_64-linux-gnu/
├── libncurses.so.5 -> libncurses.so.5.9
├── libncurses.so.5.9
├── libtinfo.so.5 -> libtinfo.so.5.9
└── libtinfo.so.5.9
```

### Verify GDB starts correctly

```bash
# Running directly will fail
arm-none-eabi-gdb --version
# error while loading shared libraries: libncurses.so.5: cannot open shared object file

# Setting LD_LIBRARY_PATH allows it to run
LD_LIBRARY_PATH=~/drivers/libncurses5/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH \
  ~/st/stm32cubeide_1.18.0/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3.rel1.linux64_1.0.0.202410170706/tools/bin/arm-none-eabi-gdb --version
# Expected: GNU gdb (GNU Tools for STM32 13.3.rel1...) 14.2.90...
```

> 📝 The `.vscode/arm-gdb.sh` wrapper script in this project handles `LD_LIBRARY_PATH` automatically. No manual setup is needed in VS Code.

---

## 4. Install VS Code Extensions

Open VS Code and install the following extensions:

| Extension | ID | Purpose |
|-----------|----|---------|
| **Cortex-Debug** | `marus25.cortex-debug` | ARM Cortex-M debug support |
| **C/C++** | `ms-vscode.cpptools` | C/C++ IntelliSense |

```bash
# Or install via command line
code --install-extension marus25.cortex-debug
code --install-extension ms-vscode.cpptools
```

---

## 5. Build Firmware with CubeIDE

### 5.1 Import the Project

1. Open STM32CubeIDE
2. `File` → `Import` → `General` → `Existing Projects into Workspace`
3. Select directory: `<this-repo>/boards/stm32H7/STM32CubeIDE`
4. Check the `spinal` project and click `Finish`

### 5.2 Build

1. Right-click the `spinal` project in Project Explorer
2. Select `Build Project` (or press `Ctrl+B`)
3. Build artifacts are located in `STM32CubeIDE/Debug/`:
   - `spinal.elf` — executable with debug info
   - `spinal.bin` — raw binary firmware
   - `spinal.map` — memory map

### 5.3 Flash via CubeIDE

1. Connect the ST-Link to the board
2. In CubeIDE, click `Run` → `Debug As` → `STM32 C/C++ Application`
3. CubeIDE will flash the firmware and start debugging

> 💡 You can also use STM32CubeProgrammer to flash `.bin` or `.elf` files separately.  
`st-flash write spinal.bin 0x08000000`

---

## 6. Debug with VS Code

### 6.1 Open the Project

```bash
code <this-repo>/boards/stm32H7/
```

### 6.2 Configuration Files

The `.vscode/` directory contains the following pre-configured files:

```
.vscode/
├── arm-gdb.sh              # GDB wrapper script (loads libncurses5)
├── c_cpp_properties.json   # C/C++ IntelliSense configuration
└── launch.json             # Debug configurations (3 modes)
```

#### `arm-gdb.sh` — GDB Wrapper

Since the `libncurses5` dependency for `arm-none-eabi-gdb` is not in the default system path,
this script sets `LD_LIBRARY_PATH` before invoking GDB:

```bash
#!/bin/bash
export LD_LIBRARY_PATH="$HOME/drivers/libncurses5/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH}"
exec $HOME/st/stm32cubeide_1.18.0/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3.rel1.linux64_1.0.0.202410170706/tools/bin/arm-none-eabi-gdb "$@"
```

> ⚠️ If your STM32CubeIDE or libncurses5 installation path is different, update the paths in this script accordingly.

### 6.3 Three Debug Modes

Select from the debug panel dropdown (`Ctrl+Shift+D`):

#### Mode 1: `STM32H743 Debug (Launch)` ⭐ Recommended

**Use case**: Firmware is already flashed; you just want to debug.

**How to use**: Press `F5`.

**Flow**: VS Code auto-starts OpenOCD → connects to ST-Link → software reset → loads debug symbols → stops at `main()`.

#### Mode 2: `STM32H743 Debug (Flash + Launch)`

**Use case**: After modifying and rebuilding code, you need to flash the new firmware before debugging.

**How to use**: Build in CubeIDE first, then select this mode and press `F5`.

**Flow**: VS Code auto-starts OpenOCD → connects to ST-Link → reset → flash ELF via GDB `load` → reset → stops at `main()`.

#### Mode 3: `STM32H743 Debug (Attach to OpenOCD)`

**Use case**: You need manual control over OpenOCD, or want to troubleshoot connection issues.

**How to use**:
```bash
# Step 1: Manually start OpenOCD in a terminal
openocd -f STM32CubeIDE/openocd_h743_stlink_v2.cfg

# Step 2: After seeing "Listening on port 3333 for gdb connections",
#          select this mode in VS Code and press F5
```

### 6.4 Quick Start (TL;DR)

```bash
# 1. Make sure the ST-Link is connected
lsusb | grep STMicro
# Expected: Bus xxx Device xxx: ID 0483:3748 STMicroelectronics ST-LINK/V2

# 2. Open the project
code <this-repo>/boards/stm32H7/

# 3. Press F5, select "STM32H743 Debug (Launch)"
```

---

## 7. Debug Features

Once the debug session starts, VS Code provides the following features:

| Feature | Description |
|---------|-------------|
| 🔴 **Breakpoints** | Click on the left margin of a line number to set/remove breakpoints |
| ▶️ **Stepping** | F10 (Step Over) / F11 (Step Into) / Shift+F11 (Step Out) |
| 👁 **Variables** | View local and global variables in the VARIABLES panel |
| 📋 **Call Stack** | View the function call chain in the CALL STACK panel |
| 🔧 **Peripherals** | View peripheral registers in the XPERIPHERALS panel (via SVD file) |
| ⌨️ **GDB Commands** | Enter GDB commands directly in the Debug Console |
| 🔄 **Restart** | Click the restart button for a software reset and re-run |

### Useful Debug Console Commands

```gdb
# View CPU registers
info registers

# Examine memory at a specific address
x/16xw 0x24040000

# Print a variable
print imu_

# List threads / FreeRTOS tasks
info threads
```
## 8. Serial Communication

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

### Q1: GDB fails with `libncurses.so.5: cannot open shared object file`

**Cause**: Missing libncurses5 and libtinfo5 libraries.

**Solution**: Follow [Section 3](#3-install-gdb-dependencies-libncurses5--libtinfo5) to install them.

### Q2: OpenOCD connection fails with `Error: open failed` or `Error: libusb_open() failed`

**Cause**: Insufficient USB permissions for the ST-Link.

**Solution**:
```bash
# Add a udev rule
sudo tee /etc/udev/rules.d/99-stlink.rules << 'EOF'
ATTRS{idVendor}=="0483", ATTRS{idProduct}=="3748", MODE="0666", GROUP="plugdev"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger
```

### Q3: Flashing fails with `timeout waiting for algorithm`

**Cause**: STM32H7 dual-bank Flash architecture + no NRST connection causes OpenOCD's `flash write_image` to time out.

**Solution**: Use **Mode 1 (Launch)** for debugging only (no flashing). Flash the firmware through CubeIDE or STM32CubeProgrammer. Alternatively, use **Mode 2 (Flash + Launch)** which uses the more reliable GDB `load` command.

### Q4: VS Code shows `"main.c" not found in compile_commands.json`

**Cause**: The `compile_commands.json` generated by CubeIDE is incomplete.

**Solution**: The `c_cpp_properties.json` is already configured with all include paths and macro definitions directly. This warning does not affect debugging.

### Q5: Debugger stops inside FreeRTOS internals instead of `main()`

**Cause**: When using Attach mode, the program is already running in FreeRTOS.

**Solution**: Execute `monitor reset init` followed by `continue` in the Debug Console, or use Launch mode which resets automatically.

### Q6: STM32CubeIDE hangs at the splash/progress bar after selecting a workspace

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

Example shell alias:
```bash
alias stm32cubeide='LD_LIBRARY_PATH=$HOME/drivers/libncurses5/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH $HOME/st/stm32cubeide_1.18.0/stm32cubeide_wayland'
```

The `LD_LIBRARY_PATH` part is mainly needed by the bundled `arm-none-eabi-gdb`; it is not expected to fix the workspace splash-screen hang by itself.

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

## Project Directory Structure

```
boards/stm32H7/
├── .vscode/                    # VS Code configuration
│   ├── arm-gdb.sh              #   GDB wrapper (loads libncurses5)
│   ├── c_cpp_properties.json   #   IntelliSense configuration
│   └── launch.json             #   Debug configurations
├── Inc/                        # Header files
├── Src/                        # Source files
│   ├── main.c                  #   CubeMX-generated init code
│   └── main.cpp                #   Main program (FreeRTOS task implementations)
├── Drivers/                    # HAL drivers
├── Middlewares/                 # FreeRTOS + LwIP
├── ros_lib/                    # rosserial library
├── STM32CubeIDE/               # CubeIDE project
│   ├── Debug/                  #   Build artifacts
│   │   ├── spinal.elf          #     Executable with debug info
│   │   └── spinal.bin          #     Raw binary firmware
│   └── openocd_h743_stlink_v2.cfg  # OpenOCD configuration
└── spinal.ioc                  # CubeMX configuration file
```
