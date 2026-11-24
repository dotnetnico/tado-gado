// Settings that survive a restart, in the flash of the ESP32. The mode is not among
// them on purpose: every boot starts in monitor, so a power cycle stops all
// transmissions.
#pragma once

#include <Arduino.h>

#include "link_monitor.h"
#include "name_table.h"

struct Settings {
  uint8_t channel;
  int8_t fsctrl0;
  uint8_t pa;
  String adminToken;
  uint8_t proxied[tadogado::kMaxProxied * tadogado::kPrefixLen];
  size_t proxiedCount;
  // A tadogado::ProxyDirection per device: 0 both ways, 1 to the valve, 2 to the bridge.
  uint8_t proxiedDirection[tadogado::kMaxProxied];
  tadogado::NameTable::Entry names[tadogado::NameTable::kCapacity];
};

Settings loadSettings();
// Counts boots in flash. Each call returns a higher number than any before it.
uint32_t nextBootNumber();
void saveRadioSettings(uint8_t channel, int8_t fsctrl0);
void savePower(uint8_t pa);
void saveAdminToken(const String& token);
void saveProxied(const uint8_t* prefixes, const uint8_t* directions, size_t count);
void saveNames(const tadogado::NameTable::Entry* names);
void eraseSettings();
// A factory reset is asked for before a restart and done after it. The request is kept
// until eraseDone(), so losing power halfway does not leave half of the settings.
void requestErase();
bool eraseRequested();
void eraseDone();
