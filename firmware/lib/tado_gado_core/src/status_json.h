// The JSON the dashboard reads, from GET /api/status.
//
// Each device has "hour" and "day": its counts over the last hour and the last 24
// hours, since the latest switch of mode at most, as an array in this order:
//   uplinks, uplinks acked, uplinks unacked, downlinks, downlinks acked,
//   downlinks unacked, retransmissions, downlinks acked behind a long train,
//   downlinks unacked behind a long train, seconds the device sent as heard here,
//   seconds of wake-up trains for the device as heard here.
// "total" has the same counts since boot, with no airtime. "windows" says how many
// seconds each window covers.
#pragma once

#include <cstddef>
#include <cstdint>

#include "link_monitor.h"
#include "name_table.h"
#include "pair_manager.h"
#include "proxy_engine.h"
#include "survey.h"

namespace tadogado {

struct RadioStatus {
  bool found;
  uint8_t version;
  uint8_t channel;
  int8_t fsctrl0;
  uint32_t resets;
  uint32_t overflows;
};

struct StatusInput {
  const LinkMonitor* monitor;
  const ProxyEngine* engine;
  const Survey* survey;
  const NameTable* names;
  const PairStatus* pair;
  RadioStatus radio;
  const char* firmware;
  const char* variant;
  bool unconfirmed;  // An update that has yet to prove that it works.
  bool readOnly;
  uint32_t freeHeap;  // Bytes, now and at the lowest since boot. 0 if unknown.
  uint32_t minFreeHeap;
  uint64_t nowUs;
  uint64_t startedUs;
};

// The buffer the firmware gives writeStatusJson. A test fills every field to its widest
// and checks that the JSON fits with room to spare.
constexpr size_t kStatusJsonCapacity = 36000;

// Writes to `out`, always terminated. Returns false if `capacity` was too small.
bool writeStatusJson(const StatusInput& input, char* out, size_t capacity);

// Field readers for a flat JSON object. Enough for the small POST bodies.
bool jsonIntField(const char* json, const char* key, int& out);
bool jsonBoolField(const char* json, const char* key, bool& out);
// Copies a string value to `out`. False if it is missing or does not fit.
bool jsonStringField(const char* json, const char* key, char* out, size_t capacity);

// Parses a 5-byte device prefix, as the dashboard shows it.
bool parsePrefix(const char* hex, uint8_t out[kPrefixLen]);

}  // namespace tadogado
