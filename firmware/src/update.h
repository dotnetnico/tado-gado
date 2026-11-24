// Installs firmware that arrives over the network, in the half of the flash that is not
// running. The bootloader returns to the old firmware if the new one does not confirm
// that it works.
#pragma once

#include <cstddef>
#include <cstdint>

namespace update {

// Why no update can start now, or nullptr.
const char* blocked();

// Each returns what went wrong, or nullptr. After an error nothing has changed, and the
// next update starts with begin() again.
const char* begin(size_t length);
const char* write(const uint8_t* data, size_t length);
// Makes the new firmware the one to start. Restart after this.
const char* finish();
void abort();

// The version of the image that was written.
const char* version();
// True until this firmware, just installed, is known to work.
bool unconfirmed();
// Call often, once the network and the web server run.
void loop();
// True from a successful finish(), and when its answer has had time to leave.
bool restarting();
bool restartDue();

}  // namespace update
