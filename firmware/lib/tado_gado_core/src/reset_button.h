// The BOOT button as a factory reset, apart from the pin, so the native tests can run it.
//
// BOOT cannot be read while the board powers up: held there, the ESP32-S3 waits for a
// flash and never starts this firmware. So the press comes just after the start.
#pragma once

#include <cstdint>

namespace tadogado {

class ResetButton {
 public:
  // A press counts only if it begins this soon after the start, so a knock on a node
  // that has been running for longer does nothing.
  static constexpr uint32_t kWindowMs = 15000;
  static constexpr uint32_t kMinHoldMs = 3000;
  // The USB port drives the same pin. A serial monitor that holds it low keeps it low
  // for as long as it is open, which must not erase the settings.
  static constexpr uint32_t kMaxHoldMs = 10000;

  // Returns true when a press of the right length is released.
  bool update(bool pressed, uint32_t sinceStartMs) {
    if (pressed) {
      if (!holding_ && sinceStartMs < kWindowMs) {
        holding_ = true;
        pressedAtMs_ = sinceStartMs;
      }
      return false;
    }
    const uint32_t heldMs = sinceStartMs - pressedAtMs_;
    const bool reset = holding_ && heldMs >= kMinHoldMs && heldMs <= kMaxHoldMs;
    holding_ = false;
    return reset;
  }

  // No press can count any more.
  bool done(uint32_t sinceStartMs) const { return !holding_ && sinceStartMs >= kWindowMs; }

 private:
  bool holding_ = false;
  uint32_t pressedAtMs_ = 0;
};

}  // namespace tadogado
