// The radio task owns the SPI bus, the link monitor and the proxy. Other tasks talk
// to it through these functions, which are safe from any task.
#pragma once

#include <cstddef>
#include <cstdint>

#include "link_monitor.h"
#include "name_table.h"

namespace radio {

struct Config {
  uint8_t channel;
  int8_t fsctrl0;
  uint8_t pa;
  const uint8_t* proxied;  // Packed 5-byte prefixes.
  size_t proxiedCount;
  const tadogado::NameTable::Entry* names;  // kCapacity entries.
  const uint8_t* proxiedDirection;          // proxiedCount entries, a tadogado::ProxyDirection each.
};

// Always starts in monitor mode, whatever was configured before the restart.
void start(const Config& config);

// Writes the dashboard JSON. Returns false if the buffer was too small.
bool writeStatus(char* out, size_t capacity, bool readOnly);

// Lets nodes with the same token find each other. Call once Wi-Fi is up. With no
// token the node stays on its own. The boot number must be higher than on any
// earlier boot: the other node refuses Hellos from a lower one as replays.
void startPairing(const char* token, const uint8_t* nodeId, uint32_t bootNumber);

// True when the proxy has nothing promised or under way, so a flash write, which
// stalls both cores, cannot make it miss an ACK.
bool idle();

void requestRetune(uint8_t channel, int8_t fsctrl0);
void requestPower(uint8_t pa);
// False if two nodes have not decided their roles yet.
bool requestProxyMode(bool enabled);
bool proxyMode();
// False if the survey cannot run: it retunes the radio, so not in proxy mode.
bool requestSurvey();

// Returns false if the list of proxied devices is full. A device served in one
// direction only has the frames of the other direction go directly.
bool setProxied(const uint8_t* prefix, bool enabled, tadogado::ProxyDirection direction);
// Copies the packed prefixes to `out` and their directions to `directions`, which
// each hold kMaxProxied entries.
size_t proxiedDevices(uint8_t* out, uint8_t* directions);
// True once after the other node of a pair changed the list.
bool takeProxiedChanged();

// Returns false for an invalid name or a full table. The other node of a pair gets it.
bool setName(const uint8_t* prefix, const char* name);
// True once after the other node of a pair changed a name.
bool takeNamesChanged();
// Copies the table to `out`, which holds NameTable::kCapacity entries.
void copyNames(tadogado::NameTable::Entry* out);

}  // namespace radio
