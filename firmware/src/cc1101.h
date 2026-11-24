// CC1101 over SPI. Only ProxyEngine::tx decides to transmit, and it refuses in
// monitor mode, which is the mode every boot starts in.
#pragma once

#include <cstddef>
#include <cstdint>

#include "rx_assembler.h"

class Cc1101 : public tadogado::RadioBus {
 public:
  void begin();
  // True if a CC1101 answers on the bus.
  bool probe();
  void configure(uint8_t channel, int8_t fsctrl0, uint8_t pa);
  void retune(uint8_t channel, int8_t fsctrl0);
  void setOutputPower(uint8_t pa);
  void calibrate();
  // False if the chip lost its configuration, for example after a brownout.
  bool configurationIntact();
  uint8_t version();
  int8_t frequencyError();

  // Transmit one frame and return to RX. Blocks while it is on air. Refuses, and sets
  // the flag below, if the chip lost its configuration.
  bool send(const uint8_t* frame, size_t length);
  bool takeConfigurationLost() {
    const bool lost = configurationLost_;
    configurationLost_ = false;
    return lost;
  }

  uint8_t rxStatus() override;
  void readFifo(uint8_t* out, size_t length) override;
  void rearm() override;

 private:
  void select();
  void deselect();
  uint8_t strobe(uint8_t command);
  void writeRegister(uint8_t address, uint8_t value);
  uint8_t readRegister(uint8_t address);
  uint8_t readStatus(uint8_t address);
  uint8_t readStatusStable(uint8_t address);
  void writeFifo(const uint8_t* data, size_t length);
  bool waitForTxEnd(uint64_t deadlineUs);

  bool configurationLost_ = false;
};
