"""Check the flash settings and the marker of a merged firmware image.

An image whose bootloader expects the wrong kind of flash does not boot, and that
shows only on the board. An application without its marker cannot be installed from
the dashboard. This catches both in CI.

Usage: python check_image.py <image.bin> <dio|dout> <size, such as 8MB> <variant> <version>
"""

import re
import sys

APP_OFFSET = 0x10000
FLASH_MODES = {0: "qio", 1: "qout", 2: "dio", 3: "dout"}
FLASH_SIZES = {0: "1MB", 1: "2MB", 2: "4MB", 3: "8MB", 4: "16MB", 5: "32MB", 6: "64MB", 7: "128MB"}


def main() -> None:
    path, want_mode, want_size, want_variant, want_version = sys.argv[1:6]
    with open(path, "rb") as file:
        image = file.read()
    header = image[:4]
    if len(header) < 4 or header[0] != 0xE9:
        sys.exit(f"{path}: no ESP image header at offset 0")
    mode = FLASH_MODES.get(header[2], f"unknown {header[2]}")
    size = FLASH_SIZES.get(header[3] >> 4, f"unknown {header[3] >> 4}")
    print(f"{path}: flash mode {mode}, flash size {size}")
    if (mode, size) != (want_mode, want_size):
        sys.exit(f"{path}: expected flash mode {want_mode} and size {want_size}")

    # What firmware/lib/tado_gado_core/src/image_check.cpp and the dashboard look for.
    if image[APP_OFFSET] != 0xE9 or image[APP_OFFSET + 32 : APP_OFFSET + 36] != bytes.fromhex("3254cdab"):
        sys.exit(f"{path}: no application image at {APP_OFFSET:#x}")
    markers = re.findall(rb"tado-gado-image:([a-z0-9]{1,15}):([0-9.]{1,15})\0", image)
    print(f"{path}: markers {markers}")
    if markers != [(want_variant.encode(), want_version.encode())]:
        sys.exit(f"{path}: expected one marker of {want_variant} {want_version}")


if __name__ == "__main__":
    main()
