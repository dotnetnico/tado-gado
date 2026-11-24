// Wiring of the CC1101 on an ESP32-S3-DevKitC-1. Change these for another board.
#pragma once

namespace board {

constexpr int kPinSck = 12;
constexpr int kPinMiso = 13;
constexpr int kPinMosi = 11;
constexpr int kPinCs = 10;
constexpr int kPinGdo0 = 4;  // Goes high at the end of a CRC-OK packet.
constexpr int kPinGdo2 = 5;  // Goes high at the RX FIFO threshold.
constexpr int kPinBoot = 0;  // The BOOT button.

}  // namespace board
