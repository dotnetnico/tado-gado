#include "settings.h"

#include <Preferences.h>

#include "phy.h"

namespace {
constexpr const char* kNamespace = "tado-gado";
// Apart from the settings, so a factory reset does not lower it.
constexpr const char* kBootNamespace = "tado-gado-boot";
constexpr const char* kEraseKey = "erase";
}  // namespace

uint32_t nextBootNumber() {
  Preferences preferences;
  preferences.begin(kBootNamespace, false);
  const uint32_t boot = preferences.getUInt("count", 0) + 1;
  preferences.putUInt("count", boot);
  preferences.end();
  return boot;
}

Settings loadSettings() {
  Preferences preferences;
  preferences.begin(kNamespace, true);
  Settings settings = {};
  settings.channel = preferences.getUChar("channel", tadogado::kDefaultChannel);
  settings.fsctrl0 = preferences.getChar("fsctrl0", 0);
  settings.pa = preferences.getUChar("pa", tadogado::kDefaultPa);
  settings.adminToken = preferences.getString("token", "");
  const size_t bytes = preferences.getBytes("proxied", settings.proxied, sizeof(settings.proxied));
  settings.proxiedCount = bytes / tadogado::kPrefixLen;
  // Absent before v0.9.0: then every device is served both ways, as it was.
  preferences.getBytes("proxydir", settings.proxiedDirection, sizeof(settings.proxiedDirection));
  preferences.getBytes("names", settings.names, sizeof(settings.names));
  preferences.end();
  return settings;
}

void saveRadioSettings(uint8_t channel, int8_t fsctrl0) {
  Preferences preferences;
  preferences.begin(kNamespace, false);
  preferences.putUChar("channel", channel);
  preferences.putChar("fsctrl0", fsctrl0);
  preferences.end();
}

void savePower(uint8_t pa) {
  Preferences preferences;
  preferences.begin(kNamespace, false);
  preferences.putUChar("pa", pa);
  preferences.end();
}

void saveAdminToken(const String& token) {
  Preferences preferences;
  preferences.begin(kNamespace, false);
  preferences.putString("token", token);
  preferences.end();
}

void saveProxied(const uint8_t* prefixes, const uint8_t* directions, size_t count) {
  Preferences preferences;
  preferences.begin(kNamespace, false);
  preferences.putBytes("proxied", prefixes, count * tadogado::kPrefixLen);
  preferences.putBytes("proxydir", directions, count);
  preferences.end();
}

void saveNames(const tadogado::NameTable::Entry* names) {
  Preferences preferences;
  preferences.begin(kNamespace, false);
  preferences.putBytes("names", names, sizeof(tadogado::NameTable::Entry) * tadogado::NameTable::kCapacity);
  preferences.end();
}

void eraseSettings() {
  Preferences preferences;
  preferences.begin(kNamespace, false);
  preferences.clear();
  preferences.end();
}

void requestErase() {
  Preferences preferences;
  preferences.begin(kBootNamespace, false);
  preferences.putBool(kEraseKey, true);
  preferences.end();
}

bool eraseRequested() {
  Preferences preferences;
  preferences.begin(kBootNamespace, false);
  const bool requested = preferences.getBool(kEraseKey, false);
  preferences.end();
  return requested;
}

void eraseDone() {
  Preferences preferences;
  preferences.begin(kBootNamespace, false);
  preferences.remove(kEraseKey);
  preferences.end();
}
