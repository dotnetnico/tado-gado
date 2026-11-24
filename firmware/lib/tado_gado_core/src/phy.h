// The CC1101 register set for the Tado V3+ PHY.
//
// Source: CappyT/freedo docs/PROTOCOL.md (Apache-2.0). 49.99 kBaud 2-GFSK with
// 25.39 kHz deviation, 135 kHz RX bandwidth, 4-byte preamble, sync word 0xD391 and
// the hardware CRC on. The carrier is 863.125 MHz + CHANNR x 199.951 kHz, with a
// 26 MHz crystal. test_core checks the derived values.
#pragma once

#include <cstdint>

namespace tadogado {

struct PhyRegister {
  uint8_t address;
  uint8_t value;
};

// PATABLE[0]: about +11 dBm at 868 MHz, inside the 25 mW e.r.p. limit of the band
// with a unity-gain antenna. Lower it for an antenna with gain.
constexpr uint8_t kDefaultPa = 0xC0;

constexpr PhyRegister kPhyRegisters[] = {
    {0x00, 0x00},  // IOCFG2: GDO2 asserts at the RX FIFO threshold
    {0x02, 0x07},  // IOCFG0: GDO0 asserts on a packet with a good CRC
    {0x03, 0x47},  // FIFOTHR: ADC retention on (RX BW < 325 kHz)
    {0x04, 0xD3},  // SYNC1
    {0x05, 0x91},  // SYNC0
    {0x06, 0xFF},  // PKTLEN
    {0x07, 0x04},  // PKTCTRL1: append RSSI and LQI/CRC_OK, no address check
    {0x08, 0x05},  // PKTCTRL0: variable length, hardware CRC on, no whitening
    {0x0A, 0x1A},  // CHANNR: 26, 868.324 MHz
    {0x0B, 0x06},  // FSCTRL1: 152 kHz IF
    {0x0C, 0x00},  // FSCTRL0: per-module crystal trim, set from the dashboard
    {0x0D, 0x21},  // FREQ2: base 863.125 MHz
    {0x0E, 0x32},  // FREQ1
    {0x0F, 0x76},  // FREQ0
    {0x10, 0xAA},  // MDMCFG4: RX BW 135 kHz, DRATE_E = 10
    {0x11, 0xF8},  // MDMCFG3: DRATE_M = 248, 49.99 kBaud
    {0x12, 0x13},  // MDMCFG2: GFSK, 30/32 sync bits
    {0x13, 0x22},  // MDMCFG1: 4-byte preamble, CHANSPC_E = 2
    {0x14, 0xF8},  // MDMCFG0: CHANSPC_M = 248, 199.951 kHz
    {0x15, 0x40},  // DEVIATN: 25.39 kHz
    {0x17, 0x3F},  // MCSM1: stay in RX after RX and after TX
    {0x18, 0x08},  // MCSM0: manual calibration only
    {0x19, 0x16},  // FOCCFG
    {0x1A, 0x6C},  // BSCFG
    {0x1B, 0x43},  // AGCCTRL2
    {0x1C, 0x40},  // AGCCTRL1
    {0x1D, 0x91},  // AGCCTRL0
    {0x21, 0x56},  // FREND1
    {0x22, 0x10},  // FREND0
    {0x23, 0xE9},  // FSCAL3
    {0x24, 0x2A},  // FSCAL2
    {0x25, 0x00},  // FSCAL1
    {0x26, 0x1F},  // FSCAL0
    {0x29, 0x59},  // FSTEST
    {0x2C, 0x81},  // TEST2
    {0x2D, 0x35},  // TEST1
    {0x2E, 0x09},  // TEST0
};

constexpr uint8_t kRegIocfg0 = 0x02;
constexpr uint8_t kRegChannr = 0x0A;
constexpr uint8_t kRegFsctrl0 = 0x0C;
constexpr uint8_t kRegFreq2 = 0x0D;
constexpr uint8_t kDefaultChannel = 26;

}  // namespace tadogado
