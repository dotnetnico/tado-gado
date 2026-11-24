// Advice on the settings, from what this node measured.
//
// Fixed rules with thresholds, run when the status is written. The node never acts on
// them: the dashboard shows each one with the numbers it rests on, and the user
// decides. A rule that rests on a rate waits for kAdviceMinFrames frames, so a quiet
// hour gives no advice rather than a wrong one.
#pragma once

#include <cstddef>
#include <cstdint>

#include "link_monitor.h"
#include "name_table.h"
#include "proxy_engine.h"

namespace tadogado {

enum class AdviceLevel : uint8_t { Info, Watch, Act };

// What the numbers of an Advice mean, per kind. A field not listed is 0.
enum class AdviceKind : uint8_t {
  ProxyToValve,   // The device misses the bridge's frames. value: downlink ACK rate,
                  // total: downlinks settled, extra: rate behind long trains, or -1.
  ProxyToBridge,  // The bridge misses the device's frames. value: uplink ACK rate,
                  // total: uplinks settled.
  WeakLink,       // Not bad enough for the proxy yet. As ProxyToValve, and count: retries.
  ProxyUnneeded,  // The device often answered the bridge itself, before the valve node
                  // could deliver. count: frames it already had, total: frames handed
                  // to this node, other: failed deliveries, value: airtime last hour, s.
  CannotJudge,    // The valve node cannot tell the device's answers from the bridge
                  // node's. value: downlink ACK rate in the baseline or -1, total: its
                  // downlinks, extra: airtime last hour, s.
  Chatty,         // The device sends far more than the others. value: uplinks an hour,
                  // extra: the median valve's, total: uplinks in the window.
  DutyHigh,       // value: share of the budget used. The device is the largest user,
                  // extra: its airtime last hour, s.
  BridgeBusy,     // value: share of the time the bridge sent. The device had the most
                  // wake-up trains, extra: their seconds in the window.
  Unnamed,        // count: devices in traffic with the bridge that have no name.
  SlowAcks,       // value: ACK latency p95 in ms.
};

struct Advice {
  AdviceKind kind;
  AdviceLevel level;
  bool hasDevice;
  uint8_t device[kPrefixLen];
  uint32_t count;
  uint32_t total;
  uint32_t other;
  float value;
  float extra;
};

struct AdviceInput {
  const LinkMonitor* monitor;
  const ProxyEngine* engine;  // nullptr on a node without the proxy.
  const NameTable* names;     // nullptr if names are unknown.
  uint64_t nowUs;
};

constexpr size_t kMaxAdvice = 16;
constexpr uint32_t kAdviceMinFrames = 30;

// Fills `out`, the most urgent first. When there is more advice than room, the least
// urgent is left out. Returns how many.
size_t computeAdvice(const AdviceInput& input, Advice* out, size_t capacity);

const char* adviceKindName(AdviceKind kind);
const char* adviceLevelName(AdviceLevel level);

}  // namespace tadogado
