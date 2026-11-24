#include "advice.h"

#include <cstring>

namespace tadogado {

namespace {

// Below this share of answered frames a device needs the proxy; below the second it is
// worth watching.
constexpr float kProxyBelow = 0.8f;
constexpr float kWatchBelow = 0.9f;
// The share of the frames handed to the valve node that the device already had, above
// which the proxy does more harm than good.
constexpr uint32_t kUnneededPercent = 30;
// A device that sends this many frames an hour, and this many times the median valve.
constexpr float kChattyPerHour = 60.0f;
constexpr float kChattyTimesMedian = 4.0f;
constexpr size_t kMedianMinDevices = 3;
constexpr uint32_t kMedianMinUplinks = 5;
// Rates per hour wait for a window of a quarter of an hour.
constexpr uint64_t kMinWindowUs = 900ULL * 1000000ULL;
constexpr float kBridgeShare = 0.01f;  // What the band allows.
constexpr float kDutyInfo = 0.5f;
constexpr float kDutyWatch = 0.7f;
constexpr float kDutyAct = 0.9f;
constexpr uint32_t kUnnamedMinFrames = 10;
constexpr float kSlowAckMs = 8.0f;
constexpr size_t kSlowAckMinSamples = 20;

float rateOf(uint32_t good, uint32_t bad) { return static_cast<float>(good) / static_cast<float>(good + bad); }

// Keeps `out` sorted from most to least urgent; when full, the least urgent goes.
class AdviceList {
 public:
  AdviceList(Advice* out, size_t capacity) : out_(out), capacity_(capacity) {}

  void add(const Advice& advice) {
    size_t at = count_;
    if (count_ < capacity_) {
      count_++;
    } else if (capacity_ > 0 && advice.level > out_[capacity_ - 1].level) {
      at = capacity_ - 1;
    } else {
      return;
    }
    while (at > 0 && out_[at - 1].level < advice.level) {
      out_[at] = out_[at - 1];
      at--;
    }
    out_[at] = advice;
  }

  size_t count() const { return count_; }

 private:
  Advice* out_;
  size_t capacity_;
  size_t count_ = 0;
};

Advice make(AdviceKind kind, AdviceLevel level, const uint8_t* device) {
  Advice advice = {};
  advice.kind = kind;
  advice.level = level;
  if (device != nullptr) {
    advice.hasDevice = true;
    std::memcpy(advice.device, device, kPrefixLen);
  }
  return advice;
}

bool isBridge(const LinkMonitor& monitor, const DeviceLink& link) {
  return monitor.hasBridge() && std::memcmp(link.prefix, monitor.bridge(), kPrefixLen) == 0;
}

// How each device that is not proxied gets on with the bridge.
void adviseLinks(const LinkMonitor& monitor, AdviceList& list) {
  for (size_t i = 0; i < kMaxDevices; i++) {
    const DeviceLink& link = monitor.devices()[i];
    if (!link.used || isBridge(monitor, link) || monitor.isProxied(link.prefix)) {
      continue;
    }
    const LinkTotals day = monitor.dayTotals(link);
    const uint32_t down = day.downlinksAcked + day.downlinksUnacked;
    const uint32_t up = day.uplinksAcked + day.uplinksUnacked;
    const uint32_t longTrain = day.longTrainAcked + day.longTrainUnacked;
    const float downRate = down > 0 ? rateOf(day.downlinksAcked, day.downlinksUnacked) : 1.0f;
    const float upRate = up > 0 ? rateOf(day.uplinksAcked, day.uplinksUnacked) : 1.0f;
    const float longRate = longTrain > 0 ? rateOf(day.longTrainAcked, day.longTrainUnacked) : -1.0f;
    if (down >= kAdviceMinFrames && downRate < kProxyBelow) {
      Advice advice = make(AdviceKind::ProxyToValve, AdviceLevel::Act, link.prefix);
      advice.value = downRate;
      advice.total = down;
      advice.extra = longRate;
      list.add(advice);
    } else if (up >= kAdviceMinFrames && upRate < kProxyBelow) {
      Advice advice = make(AdviceKind::ProxyToBridge, AdviceLevel::Act, link.prefix);
      advice.value = upRate;
      advice.total = up;
      list.add(advice);
    } else if (down >= kAdviceMinFrames && downRate < kWatchBelow) {
      Advice advice = make(AdviceKind::WeakLink, AdviceLevel::Watch, link.prefix);
      advice.value = downRate;
      advice.total = down;
      advice.extra = longRate;
      advice.count = day.retransmissions;
      list.add(advice);
    }
  }
}

// Only the valve node of a pair knows whether a proxied device answered the bridge
// before it could deliver.
void adviseProxied(const LinkMonitor& monitor, const ProxyEngine& engine, uint64_t nowUs, AdviceList& list) {
  if (engine.role() != Role::Valve || !engine.proxyMode()) {
    return;
  }
  for (size_t i = 0; i < kMaxDevices; i++) {
    const DeviceLink& link = monitor.devices()[i];
    if (!link.used || !monitor.proxiesDownlink(link.prefix)) {
      continue;
    }
    const float airtimeS = static_cast<float>(link.airtime.usedUs(nowUs)) / 1e6f;
    if (engine.canTellOwnAnswer(link.prefix)) {
      const uint32_t handed = link.handedDownlinks;
      if (handed >= kAdviceMinFrames &&
          static_cast<uint64_t>(link.alreadyAnswered) * 100 >= static_cast<uint64_t>(handed) * kUnneededPercent) {
        Advice advice = make(AdviceKind::ProxyUnneeded, AdviceLevel::Act, link.prefix);
        advice.count = link.alreadyAnswered;
        advice.total = handed;
        advice.other = link.deliveryFailures;
        advice.value = airtimeS;
        list.add(advice);
      }
      continue;
    }
    Advice advice = make(AdviceKind::CannotJudge, AdviceLevel::Info, link.prefix);
    const uint32_t before = link.baseline.downlinksAcked + link.baseline.downlinksUnacked;
    advice.total = before;
    advice.value = before > 0 ? rateOf(link.baseline.downlinksAcked, link.baseline.downlinksUnacked) : -1.0f;
    advice.extra = airtimeS;
    list.add(advice);
  }
}

void adviseChatty(const LinkMonitor& monitor, uint64_t nowUs, AdviceList& list) {
  const uint64_t windowUs = monitor.dayCoveredUs(nowUs);
  if (windowUs < kMinWindowUs) {
    return;
  }
  const float hours = static_cast<float>(windowUs) / 3.6e9f;
  float rates[kMaxDevices];
  size_t count = 0;
  for (size_t i = 0; i < kMaxDevices; i++) {
    const DeviceLink& link = monitor.devices()[i];
    if (!link.used || isBridge(monitor, link)) {
      continue;
    }
    const uint32_t uplinks = monitor.dayTotals(link).uplinks;
    if (uplinks < kMedianMinUplinks) {
      continue;
    }
    // Insertion sort: a few dozen values at most.
    const float rate = static_cast<float>(uplinks) / hours;
    size_t at = count++;
    while (at > 0 && rates[at - 1] > rate) {
      rates[at] = rates[at - 1];
      at--;
    }
    rates[at] = rate;
  }
  if (count < kMedianMinDevices) {
    return;
  }
  const float median = rates[count / 2];
  for (size_t i = 0; i < kMaxDevices; i++) {
    const DeviceLink& link = monitor.devices()[i];
    if (!link.used || isBridge(monitor, link)) {
      continue;
    }
    const uint32_t uplinks = monitor.dayTotals(link).uplinks;
    const float rate = static_cast<float>(uplinks) / hours;
    if (uplinks >= kAdviceMinFrames && rate >= kChattyPerHour && rate >= kChattyTimesMedian * median) {
      Advice advice = make(AdviceKind::Chatty, AdviceLevel::Watch, link.prefix);
      advice.value = rate;
      advice.extra = median;
      advice.total = uplinks;
      list.add(advice);
    }
  }
}

void adviseDuty(const LinkMonitor& monitor, const ProxyEngine& engine, uint64_t nowUs, AdviceList& list) {
  const float used = static_cast<float>(engine.duty().usedUs(nowUs)) / static_cast<float>(DutyCycleLimiter::kBudgetUs);
  if (used < kDutyInfo) {
    return;
  }
  const AdviceLevel level = used >= kDutyAct     ? AdviceLevel::Act
                            : used >= kDutyWatch ? AdviceLevel::Watch
                                                 : AdviceLevel::Info;
  const DeviceLink* top = nullptr;
  uint64_t topUs = 0;
  for (size_t i = 0; i < kMaxDevices; i++) {
    const DeviceLink& link = monitor.devices()[i];
    const uint64_t spent = link.used ? link.airtime.usedUs(nowUs) : 0;
    if (spent > topUs) {
      top = &link;
      topUs = spent;
    }
  }
  Advice advice = make(AdviceKind::DutyHigh, level, top != nullptr ? top->prefix : nullptr);
  advice.value = used;
  advice.extra = static_cast<float>(topUs) / 1e6f;
  list.add(advice);
}

void adviseBridge(const LinkMonitor& monitor, uint64_t nowUs, AdviceList& list) {
  const uint64_t windowUs = monitor.dayCoveredUs(nowUs);
  const DeviceLink* bridge = monitor.hasBridge() ? monitor.findLink(monitor.bridge()) : nullptr;
  if (bridge == nullptr || windowUs < kMinWindowUs) {
    return;
  }
  const float share = static_cast<float>(monitor.dayTotals(*bridge).airtimeUs) / static_cast<float>(windowUs);
  if (share <= kBridgeShare) {
    return;
  }
  const DeviceLink* busiest = nullptr;
  uint64_t busiestUs = 0;
  for (size_t i = 0; i < kMaxDevices; i++) {
    const DeviceLink& link = monitor.devices()[i];
    if (!link.used || isBridge(monitor, link)) {
      continue;
    }
    const uint64_t trains = monitor.dayTotals(link).trainUs;
    if (trains > busiestUs) {
      busiest = &link;
      busiestUs = trains;
    }
  }
  Advice advice = make(AdviceKind::BridgeBusy, AdviceLevel::Info, busiest != nullptr ? busiest->prefix : nullptr);
  advice.value = share;
  advice.extra = static_cast<float>(busiestUs) / 1e6f;
  list.add(advice);
}

void adviseNames(const LinkMonitor& monitor, const NameTable& names, AdviceList& list) {
  uint32_t unnamed = 0;
  for (size_t i = 0; i < kMaxDevices; i++) {
    const DeviceLink& link = monitor.devices()[i];
    if (!link.used || isBridge(monitor, link) || names.find(link.prefix) != nullptr) {
      continue;
    }
    const LinkTotals day = monitor.dayTotals(link);
    if (day.uplinks + day.downlinks >= kUnnamedMinFrames) {
      unnamed++;
    }
  }
  if (unnamed > 0) {
    Advice advice = make(AdviceKind::Unnamed, AdviceLevel::Info, nullptr);
    advice.count = unnamed;
    list.add(advice);
  }
}

void adviseLatency(const ProxyEngine& engine, AdviceList& list) {
  AckLatency latency;
  if (engine.ackLatency(latency) && latency.samples >= kSlowAckMinSamples && latency.p95Ms > kSlowAckMs) {
    Advice advice = make(AdviceKind::SlowAcks, AdviceLevel::Act, nullptr);
    advice.value = latency.p95Ms;
    list.add(advice);
  }
}

}  // namespace

size_t computeAdvice(const AdviceInput& input, Advice* out, size_t capacity) {
  AdviceList list(out, capacity);
  const LinkMonitor& monitor = *input.monitor;
  adviseLinks(monitor, list);
  adviseChatty(monitor, input.nowUs, list);
  adviseBridge(monitor, input.nowUs, list);
  if (input.engine != nullptr) {
    adviseProxied(monitor, *input.engine, input.nowUs, list);
    adviseDuty(monitor, *input.engine, input.nowUs, list);
    adviseLatency(*input.engine, list);
  }
  if (input.names != nullptr) {
    adviseNames(monitor, *input.names, list);
  }
  return list.count();
}

const char* adviceKindName(AdviceKind kind) {
  switch (kind) {
    case AdviceKind::ProxyToValve:
      return "proxy_to_valve";
    case AdviceKind::ProxyToBridge:
      return "proxy_to_bridge";
    case AdviceKind::WeakLink:
      return "weak_link";
    case AdviceKind::ProxyUnneeded:
      return "proxy_unneeded";
    case AdviceKind::CannotJudge:
      return "cannot_judge";
    case AdviceKind::Chatty:
      return "chatty";
    case AdviceKind::DutyHigh:
      return "duty_high";
    case AdviceKind::BridgeBusy:
      return "bridge_busy";
    case AdviceKind::Unnamed:
      return "unnamed";
    case AdviceKind::SlowAcks:
      return "slow_acks";
  }
  return "unknown";
}

const char* adviceLevelName(AdviceLevel level) {
  switch (level) {
    case AdviceLevel::Act:
      return "act";
    case AdviceLevel::Watch:
      return "watch";
    case AdviceLevel::Info:
      return "info";
  }
  return "info";
}

}  // namespace tadogado
