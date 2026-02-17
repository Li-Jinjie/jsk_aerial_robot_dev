#!/bin/bash
export LD_LIBRARY_PATH="/home/wentao/drivers/libncurses5/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH}"
exec /home/wentao/st/stm32cubeide_1.18.0/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3.rel1.linux64_1.0.0.202410170706/tools/bin/arm-none-eabi-gdb "$@"
