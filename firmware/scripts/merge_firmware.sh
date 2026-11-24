#!/usr/bin/env bash
# Merge the bootloader, partition table and application into one image, which can
# be flashed at offset 0 without a toolchain. Run from firmware/ after `pio run`.
#
# The flash mode, speed and size stay as the build wrote them into the bootloader: an
# octal flash board needs the octal bootloader, and overriding its header breaks it.
set -euo pipefail

environment="${1:?usage: merge_firmware.sh <environment> <output.bin>}"
output="${2:?usage: merge_firmware.sh <environment> <output.bin>}"
build=".pio/build/$environment"
packages="${PLATFORMIO_CORE_DIR:-$HOME/.platformio}/packages"

python "$packages/tool-esptoolpy/esptool.py" --chip esp32s3 merge_bin \
  --output "$output" \
  --flash_mode keep --flash_freq keep --flash_size keep \
  0x0000 "$build/bootloader.bin" \
  0x8000 "$build/partitions.bin" \
  0xe000 "$packages/framework-arduinoespressif32/tools/partitions/boot_app0.bin" \
  0x10000 "$build/firmware.bin"
