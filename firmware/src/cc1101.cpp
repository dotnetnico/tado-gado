#include "cc1101.h"

#include <Arduino.h>
#include <SPI.h>

#include "board.h"
#include "phy.h"

namespace {

constexpr uint8_t kReadSingle = 0x80;
constexpr uint8_t kReadBurst = 0xC0;
constexpr uint8_t kWriteBurst = 0x40;
constexpr uint8_t kFifo = 0x3F;
constexpr uint8_t kPaTable = 0x3E;

constexpr uint8_t kStrobeReset = 0x30;
constexpr uint8_t kStrobeCalibrate = 0x33;
constexpr uint8_t kStrobeRx = 0x34;
constexpr uint8_t kStrobeTx = 0x35;
constexpr uint8_t kStrobeIdle = 0x36;
constexpr uint8_t kStrobeFlushRx = 0x3A;
constexpr uint8_t kStrobeFlushTx = 0x3B;

constexpr uint8_t kStatusPartNumber = 0x30;
constexpr uint8_t kStatusVersion = 0x31;
constexpr uint8_t kStatusFreqEst = 0x32;
constexpr uint8_t kStatusMarcState = 0x35;
constexpr uint8_t kStatusTxBytes = 0x3A;
constexpr uint8_t kStatusRxBytes = 0x3B;

constexpr uint8_t kMarcStateMask = 0x1F;
constexpr uint8_t kMarcTx = 0x13;
constexpr uint8_t kMarcTxEnd = 0x14;
constexpr uint8_t kMarcTxUnderflow = 0x16;
constexpr uint8_t kFifoErrorBit = 0x80;
constexpr uint8_t kFifoBytesMask = 0x7F;
constexpr size_t kFifoSize = 64;

constexpr int kStableReadAttempts = 8;
constexpr uint32_t kReadyTimeoutUs = 1000;
constexpr uint64_t kTxTimeoutUs = 200000;  // A 127-byte frame takes 22 ms on air.

// The default is 500 kHz in many examples, but the FIFO read must stay well ahead
// of the air rate. The chip allows 6.5 MHz for bursts.
const SPISettings kSpiSettings(4000000, MSBFIRST, SPI_MODE0);

}  // namespace

void Cc1101::begin() {
  pinMode(board::kPinCs, OUTPUT);
  digitalWrite(board::kPinCs, HIGH);
  SPI.begin(board::kPinSck, board::kPinMiso, board::kPinMosi, -1);
}

void Cc1101::select() {
  SPI.beginTransaction(kSpiSettings);
  digitalWrite(board::kPinCs, LOW);
  // The chip pulls SO low when its crystal runs. Bounded, so a missing chip cannot
  // hang the radio task.
  const uint32_t start = micros();
  while (digitalRead(board::kPinMiso) == HIGH && micros() - start < kReadyTimeoutUs) {
  }
}

void Cc1101::deselect() {
  digitalWrite(board::kPinCs, HIGH);
  SPI.endTransaction();
}

uint8_t Cc1101::strobe(uint8_t command) {
  select();
  const uint8_t status = SPI.transfer(command);
  deselect();
  return status;
}

void Cc1101::writeRegister(uint8_t address, uint8_t value) {
  select();
  SPI.transfer(address);
  SPI.transfer(value);
  deselect();
}

uint8_t Cc1101::readRegister(uint8_t address) {
  select();
  SPI.transfer(address | kReadSingle);
  const uint8_t value = SPI.transfer(0);
  deselect();
  return value;
}

uint8_t Cc1101::readStatus(uint8_t address) {
  select();
  SPI.transfer(address | kReadBurst);
  const uint8_t value = SPI.transfer(0);
  deselect();
  return value;
}

// Errata: a read during an update of a live register can return a mix of the old
// and the new value.
uint8_t Cc1101::readStatusStable(uint8_t address) {
  uint8_t value = readStatus(address);
  for (int i = 0; i < kStableReadAttempts; i++) {
    const uint8_t again = readStatus(address);
    if (again == value) {
      return value;
    }
    value = again;
  }
  return value;
}

bool Cc1101::probe() {
  strobe(kStrobeReset);
  delay(2);
  const uint8_t part = readStatus(kStatusPartNumber);
  const uint8_t chipVersion = readStatus(kStatusVersion);
  return part == 0x00 && chipVersion != 0x00 && chipVersion != 0xFF;
}

void Cc1101::configure(uint8_t channel, int8_t fsctrl0, uint8_t pa) {
  strobe(kStrobeReset);
  delay(2);
  for (const tadogado::PhyRegister& reg : tadogado::kPhyRegisters) {
    writeRegister(reg.address, reg.value);
  }
  setOutputPower(pa);
  retune(channel, fsctrl0);
}

void Cc1101::setOutputPower(uint8_t pa) { writeRegister(kPaTable | kWriteBurst, pa); }

void Cc1101::retune(uint8_t channel, int8_t fsctrl0) {
  strobe(kStrobeIdle);
  writeRegister(tadogado::kRegChannr, channel);
  writeRegister(tadogado::kRegFsctrl0, static_cast<uint8_t>(fsctrl0));
  calibrate();
}

void Cc1101::calibrate() {
  strobe(kStrobeIdle);
  strobe(kStrobeCalibrate);
  delay(2);
  rearm();
}

// IOCFG0 and FREQ2 differ from their reset values (0x3F and 0x1E), so a chip that
// reset by itself fails this.
bool Cc1101::configurationIntact() {
  size_t matched = 0;
  for (const tadogado::PhyRegister& reg : tadogado::kPhyRegisters) {
    if (reg.address == tadogado::kRegIocfg0 || reg.address == tadogado::kRegFreq2) {
      if (readRegister(reg.address) != reg.value) {
        return false;
      }
      matched++;
    }
  }
  return matched == 2;
}

uint8_t Cc1101::version() { return readStatus(kStatusVersion); }

int8_t Cc1101::frequencyError() { return static_cast<int8_t>(readStatus(kStatusFreqEst)); }

uint8_t Cc1101::rxStatus() { return readStatusStable(kStatusRxBytes); }

void Cc1101::readFifo(uint8_t* out, size_t length) {
  select();
  SPI.transfer(kFifo | kReadBurst);
  for (size_t i = 0; i < length; i++) {
    out[i] = SPI.transfer(0);
  }
  deselect();
}

void Cc1101::rearm() {
  strobe(kStrobeIdle);
  strobe(kStrobeFlushRx);
  strobe(kStrobeRx);
}

void Cc1101::writeFifo(const uint8_t* data, size_t length) {
  select();
  SPI.transfer(kFifo | kWriteBurst);
  for (size_t i = 0; i < length; i++) {
    SPI.transfer(data[i]);
  }
  deselect();
}

// MCSM1 returns the chip to RX by itself, so it never shows IDLE: a transmission is
// over once the chip has left TX with an empty FIFO.
bool Cc1101::waitForTxEnd(uint64_t deadlineUs) {
  while (static_cast<uint64_t>(esp_timer_get_time()) < deadlineUs) {
    const uint8_t state = readStatusStable(kStatusMarcState) & kMarcStateMask;
    if (state == kMarcTxUnderflow) {
      return false;
    }
    if (state != kMarcTx && state != kMarcTxEnd && (readStatusStable(kStatusTxBytes) & kFifoBytesMask) == 0) {
      return true;
    }
  }
  return false;
}

bool Cc1101::send(const uint8_t* frame, size_t length) {
  if (length == 0 || length > tadogado::kMaxFrameLen) {
    return false;
  }
  // A chip that reset by itself would transmit at its default 800 MHz.
  if (!configurationIntact()) {
    configurationLost_ = true;
    return false;
  }
  // SIDLE keeps a frame cut off in mid-air in the RX FIFO, and the answer to this
  // frame lands behind it, so flush now, while the chip is idle.
  select();
  SPI.transfer(kStrobeIdle);
  SPI.transfer(kStrobeFlushRx);
  SPI.transfer(kStrobeFlushTx);
  deselect();

  const uint8_t lengthByte = static_cast<uint8_t>(length);
  const size_t firstChunk = length < kFifoSize - 1 ? length : kFifoSize - 1;
  select();
  SPI.transfer(kFifo | kWriteBurst);
  SPI.transfer(lengthByte);
  for (size_t i = 0; i < firstChunk; i++) {
    SPI.transfer(frame[i]);
  }
  deselect();
  strobe(kStrobeTx);

  const uint64_t deadline = esp_timer_get_time() + kTxTimeoutUs;
  size_t sent = firstChunk;
  bool ok = true;
  while (ok && sent < length) {
    const uint8_t status = readStatusStable(kStatusTxBytes);
    if ((status & kFifoErrorBit) || static_cast<uint64_t>(esp_timer_get_time()) > deadline) {
      ok = false;
      break;
    }
    const size_t used = status & kFifoBytesMask;
    if (used >= kFifoSize) {
      continue;  // A read above 64 is a corrupt one: read again.
    }
    const size_t room = kFifoSize - used;
    const size_t chunk = length - sent < room ? length - sent : room;
    writeFifo(frame + sent, chunk);
    sent += chunk;
  }
  if (ok) {
    ok = waitForTxEnd(deadline);
  }
  if (!ok) {
    strobe(kStrobeIdle);
    strobe(kStrobeFlushTx);
    rearm();
  }
  return ok;
}
