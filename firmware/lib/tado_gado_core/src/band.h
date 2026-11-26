// Where the node may send. The proxy sends only in the 868.0-868.6 MHz band, where a
// short-range device may use 25 mW e.r.p. for 1% of the time (ERC Rec 70-03, annex 1).
// The CC1101 tunes from 863 to 870 MHz, but most of that range allows 0.1% or less,
// and the duty-cycle budget of the proxy is the one of this band.
#pragma once

#include <cstdint>

#include "phy.h"

namespace tadogado {

constexpr int64_t kBandLowHz = 868000000;
constexpr int64_t kBandHighHz = 868600000;
// Half the width of a frame on air. Carson's rule gives about 50 kHz for 49.99 kBaud
// 2-GFSK with 25.39 kHz deviation; the rest covers a crystal 10 ppm off.
constexpr int64_t kHalfWidthHz = 60000;

constexpr uint8_t phyValue(uint8_t address) {
  for (const PhyRegister& reg : kPhyRegisters) {
    if (reg.address == address) {
      return reg.value;
    }
  }
  return 0;
}

// The carrier for a channel and a crystal trim, from the registers and a 26 MHz crystal.
constexpr int64_t carrierHz(uint8_t channel, int8_t fsctrl0) {
  constexpr int64_t kXtalHz = 26000000;
  const int64_t freq = static_cast<int64_t>(phyValue(0x0D)) << 16 | phyValue(0x0E) << 8 | phyValue(0x0F);
  // In units of the crystal / 2^18: the channel spacing has 2^(CHANSPC_E - 2) of the
  // 2^16 units of FREQ, and FSCTRL0 counts in steps of the crystal / 2^14.
  const int64_t spacing = (256 + phyValue(0x14)) * (int64_t{1} << (phyValue(0x13) & 0x03));
  const int64_t units = freq * 4 + spacing * channel + int64_t{16} * fsctrl0;
  return kXtalHz * units / (int64_t{1} << 18);
}

constexpr bool transmitAllowed(uint8_t channel, int8_t fsctrl0) {
  const int64_t carrier = carrierHz(channel, fsctrl0);
  return carrier - kHalfWidthHz >= kBandLowHz && carrier + kHalfWidthHz <= kBandHighHz;
}

static_assert(transmitAllowed(kDefaultChannel, 0), "the default channel must be in the band");

}  // namespace tadogado
