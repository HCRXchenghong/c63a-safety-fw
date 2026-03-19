## STM32CubeIDE migration

This repository has been adapted so that a fresh STM32CubeIDE project generated from `WHEELTEC.ioc` can build the existing custom code without manually adding `WHEELTEC_APP` or `WHEELTEC_BSP` as source folders.

What changed:
- `WHEELTEC.ioc` now targets `STM32CubeIDE`.
- `Core/Inc/cmsis_armcc.h` provides a GCC-safe compatibility shim for legacy ARMCC includes.
- `Core/Inc` contains wrapper headers for custom APP/BSP headers.
- `Core/Src` contains `cubide_wrap_*.c` translation units that pull custom APP/BSP/USB host sources into the default CubeIDE source tree.
- A GCC FreeRTOS port directory must exist at `Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F`.

What you still need to do in STM32CubeIDE:
1. Open `WHEELTEC.ioc`.
2. Let CubeIDE install the STM32F4 firmware package if prompted.
3. Click `Generate Code` once so CubeIDE creates `.project`, `.cproject`, the GCC startup file, and the linker script.
4. Build the project.

Notes:
- Do not manually add `WHEELTEC_APP/*.c`, `WHEELTEC_BSP/*.c`, or the extra USB host gamepad sources to the CubeIDE project again. The `cubide_wrap_*.c` files already compile them.
- If CubeIDE later imports the original custom folders directly, remove the corresponding `cubide_wrap_*.c` files to avoid duplicate symbols.
