#!/usr/bin/env bash
#
# build.sh -- rebuild the motor-control APP firmware and emit BOTH:
#   Debug/Generic_motor_controller.elf   (SWD flash / debug)
#   Debug/Generic_motor_controller.bin   (OTA bootload image -- flash straight on)
#
# Uses the STM32CubeIDE make + the STM32CubeCLT arm-none-eabi toolchain (globbed so
# a version bump doesn't break it). Run from anywhere:  ./build.sh
#
# NOTE: this builds the APP, not the bootloader. The bootloader has its own build
# (boot/, see ADR-064). The .bin is the relocated app image (linked at 0x08008800).
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

TC_OBJCOPY="$(ls /c/ST/STM32CubeCLT_*/GNU-tools-for-STM32/bin/arm-none-eabi-objcopy.exe 2>/dev/null | sort -V | tail -1)"
TC_BIN="$(dirname "$TC_OBJCOPY")"
MAKE="$(ls /c/ST/STM32CubeIDE_*/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.make.win32_*/tools/bin/make.exe 2>/dev/null | sort -V | tail -1)"

[ -x "$TC_OBJCOPY" ] || { echo "ERROR: arm-none-eabi-objcopy not found under /c/ST/STM32CubeCLT_*" >&2; exit 1; }
[ -x "$MAKE" ]       || { echo "ERROR: CubeIDE make not found under /c/ST/STM32CubeIDE_*" >&2; exit 1; }

export PATH="$TC_BIN:$PATH"   # subdir.mk calls arm-none-eabi-gcc by bare name

echo "==> make -C Debug all"
"$MAKE" -C "$ROOT/Debug" -j"$(nproc 2>/dev/null || echo 4)" all

ELF="$ROOT/Debug/Generic_motor_controller.elf"
BIN="$ROOT/Debug/Generic_motor_controller.bin"
"$TC_OBJCOPY" -O binary "$ELF" "$BIN"

echo "==> built:"
ls -la "$ELF" "$BIN"
python -c "import zlib,sys; d=open(sys.argv[1],'rb').read(); print(f'    app.bin: {len(d)} bytes, crc32=0x{zlib.crc32(d)&0xFFFFFFFF:08X} (matches OTA VERIFY / 0x1F56)')" "$BIN" 2>/dev/null || true
