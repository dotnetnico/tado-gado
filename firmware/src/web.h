// Dashboard and HTTP API.
#pragma once

#include <Arduino.h>

namespace web {

// `hostName` is the mDNS name without ".local". Requests for other host names are
// refused, which stops DNS rebinding. Returns false if the server did not start.
bool begin(const String& adminToken, const String& hostName);
// Writes changed settings to flash, and restarts after an update. Call it often.
void loop();

}  // namespace web
