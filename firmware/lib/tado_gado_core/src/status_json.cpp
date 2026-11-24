#include "status_json.h"

#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace tadogado {

namespace {

// printf into a fixed buffer. Keeps writing nothing once it has run out.
class Writer {
 public:
  Writer(char* out, size_t capacity) : out_(out), capacity_(capacity) {
    if (capacity > 0) {
      out[0] = '\0';
    }
  }

  void print(const char* format, ...) __attribute__((format(printf, 2, 3))) {
    if (!ok_) {
      return;
    }
    va_list args;
    va_start(args, format);
    const int written = vsnprintf(out_ + used_, capacity_ - used_, format, args);
    va_end(args);
    if (written < 0 || static_cast<size_t>(written) >= capacity_ - used_) {
      ok_ = false;
      out_[used_] = '\0';
      return;
    }
    used_ += static_cast<size_t>(written);
  }

  void prefix(const uint8_t* value) {
    print("\"%02x%02x%02x%02x%02x\"", value[0], value[1], value[2], value[3], value[4]);
  }

  void rate(uint32_t good, uint32_t bad) {
    const uint32_t total = good + bad;
    if (total == 0) {
      print("null");
    } else {
      print("%.3f", static_cast<double>(good) / total);
    }
  }

  bool ok() const { return ok_; }

 private:
  char* out_;
  size_t capacity_;
  size_t used_ = 0;
  bool ok_ = true;
};

double seconds(uint64_t microseconds) { return static_cast<double>(microseconds) / 1000000.0; }

void writeWindow(Writer& w, const LinkTotals& t) {
  w.print("[");
  for (size_t i = 0; i < kLinkCountFieldCount; i++) {
    w.print("%lu,", static_cast<unsigned long>(t.*kLinkTotalFields[i]));
  }
  w.print("%.2f,%.2f]", seconds(t.airtimeUs), seconds(t.trainUs));
}

void writeTrains(Writer& w, const TrainTotals& t) {
  w.print("[%.2f,%.2f,%.2f]", seconds(t.bridgeUs), seconds(t.peerUs), seconds(t.unknownUs));
}

// The counts since boot, in the shape of a window. Airtime is not kept that long.
LinkTotals sinceBoot(const DeviceLink& d) {
  LinkTotals t = {};
  t.uplinks = d.uplinks;
  t.uplinksAcked = d.uplinksAcked;
  t.uplinksUnacked = d.uplinksUnacked;
  t.downlinks = d.downlinks;
  t.downlinksAcked = d.downlinksAcked;
  t.downlinksUnacked = d.downlinksUnacked;
  t.retransmissions = d.retransmissions;
  t.longTrainAcked = d.longTrainAcked;
  t.longTrainUnacked = d.longTrainUnacked;
  return t;
}

// The time a window covers: from the start of its oldest bucket or the latest reset.
uint64_t coveredUs(uint64_t nowUs, uint64_t sinceUs, uint64_t bucketUs, size_t buckets) {
  const uint64_t index = nowUs / bucketUs;
  const uint64_t start = index >= buckets - 1 ? (index - (buckets - 1)) * bucketUs : 0;
  const uint64_t from = start > sinceUs ? start : sinceUs;
  return nowUs > from ? nowUs - from : 0;
}

const char* proxyDirection(const LinkMonitor& monitor, const uint8_t* prefix) {
  if (!monitor.isProxied(prefix)) {
    return "null";
  }
  if (!monitor.proxiesUplink(prefix)) {
    return "\"downlink\"";
  }
  return monitor.proxiesDownlink(prefix) ? "\"both\"" : "\"uplink\"";
}

void writePair(Writer& w, const PairStatus* pair) {
  if (pair == nullptr || !pair->enabled) {
    w.print("null");
    return;
  }
  w.print("{\"peer\":");
  if (pair->hasPeer) {
    const uint8_t* id = pair->peerId;
    w.print("\"%02x%02x%02x%02x%02x%02x\"", id[0], id[1], id[2], id[3], id[4], id[5]);
  } else {
    w.print("null");
  }
  w.print(",\"peer_alive\":%s,\"roles_decided\":%s,\"peer_restarts\":%lu,\"rejected\":%lu",
          pair->peerAlive ? "true" : "false", pair->rolesDecided ? "true" : "false",
          static_cast<unsigned long>(pair->peerRestarts), static_cast<unsigned long>(pair->rejected));
  if (pair->hasOwnRssi) {
    w.print(",\"own_bridge_rssi\":%.1f", pair->ownRssi);
  } else {
    w.print(",\"own_bridge_rssi\":null");
  }
  if (pair->hasPeerRssi) {
    w.print(",\"peer_bridge_rssi\":%.1f}", pair->peerRssi);
  } else {
    w.print(",\"peer_bridge_rssi\":null}");
  }
}

void writeDevice(Writer& w, const DeviceLink& d, const LinkMonitor& monitor, const ProxyEngine* engine,
                 const NameTable* names, uint64_t nowUs) {
  const bool isBridge = monitor.hasBridge() && std::memcmp(d.prefix, monitor.bridge(), kPrefixLen) == 0;
  w.print("{\"device\":");
  w.prefix(d.prefix);
  const char* name = names != nullptr ? names->find(d.prefix) : nullptr;
  if (name != nullptr) {
    w.print(",\"name\":\"%s\"", name);  // NameTable refuses what would need escaping.
  } else {
    w.print(",\"name\":null");
  }
  if (d.downlinks > 0) {
    w.print(",\"downlink_age_s\":%.1f", seconds(nowUs - d.lastDownlinkUs));
  } else {
    w.print(",\"downlink_age_s\":null");
  }
  w.print(",\"kind\":\"%s\",\"proxied\":%s,\"proxy_direction\":%s,\"frames\":%lu", isBridge ? "bridge" : "device",
          monitor.isProxied(d.prefix) ? "true" : "false", proxyDirection(monitor, d.prefix),
          static_cast<unsigned long>(d.frames));
  if (d.frames > 0) {
    w.print(",\"rssi_dbm\":%.1f,\"lqi\":%.1f,\"seconds_since_seen\":%.1f", d.rssiDbm, d.lqi,
            seconds(nowUs - d.lastSeenUs));
  } else {
    w.print(",\"rssi_dbm\":null,\"lqi\":null,\"seconds_since_seen\":null");
  }
  w.print(",\"uplinks\":%lu,\"uplink_ack_rate\":", static_cast<unsigned long>(d.uplinks));
  w.rate(d.uplinksAcked, d.uplinksUnacked);
  w.print(",\"downlinks\":%lu,\"downlink_ack_rate\":", static_cast<unsigned long>(d.downlinks));
  w.rate(d.downlinksAcked, d.downlinksUnacked);
  w.print(",\"long_train_ack_rate\":");
  w.rate(d.longTrainAcked, d.longTrainUnacked);
  w.print(",\"acks_from_peer\":%lu", static_cast<unsigned long>(d.ackedByPeer));
  // Null where the valve node cannot tell the device's own answers from the bridge node's.
  if (engine != nullptr && engine->role() == Role::Valve && engine->canTellOwnAnswer(d.prefix)) {
    w.print(",\"already_answered\":%lu", static_cast<unsigned long>(d.alreadyAnswered));
  } else {
    w.print(",\"already_answered\":null");
  }
  w.print(",\"hour\":");
  writeWindow(w, monitor.hourTotals(d));
  w.print(",\"day\":");
  writeWindow(w, monitor.dayTotals(d));
  w.print(",\"total\":");
  writeWindow(w, sinceBoot(d));
  w.print(
      ",\"retransmissions\":%lu,\"proxied_uplinks\":%lu,\"proxied_downlinks\":%lu,"
      "\"delivery_failures\":%lu,\"airtime_s\":%.2f,\"has_anchor\":%s}",
      static_cast<unsigned long>(d.retransmissions), static_cast<unsigned long>(d.proxiedUplinks),
      static_cast<unsigned long>(d.proxiedDownlinks), static_cast<unsigned long>(d.deliveryFailures),
      seconds(d.airtime.usedUs(nowUs)), engine != nullptr && engine->anchors().contains(d.prefix) ? "true" : "false");
}

void writeFrame(Writer& w, const RecentFrame& f, uint64_t nowUs) {
  w.print("{\"age_s\":%.1f,\"class\":\"%s\",\"length\":%u,\"rssi_dbm\":%.1f,\"lqi\":%u,\"src\":",
          seconds(nowUs - f.timestampUs), frameClassName(f.frameClass), f.length, f.rssiDbm, f.lqi);
  if (f.hasAddresses) {
    w.prefix(f.src);
  } else {
    w.print("null");
  }
  w.print(",\"dst\":");
  if (f.hasDestination) {
    w.prefix(f.dst);
  } else {
    w.print("null");
  }
  if (f.hasSeq) {
    w.print(",\"seq\":%u}", f.seq);
  } else {
    w.print(",\"seq\":null}");
  }
}

void writeSurvey(Writer& w, const Survey& survey) {
  w.print("{\"running\":%s,\"finished\":%s,\"step\":%u,\"steps\":%u,\"results\":[", survey.running() ? "true" : "false",
          survey.finished() ? "true" : "false", static_cast<unsigned>(survey.step()),
          static_cast<unsigned>(Survey::kResults));
  const size_t count = survey.resultCount();
  for (size_t i = 0; i < count; i++) {
    const SurveyResult& r = survey.result(i);
    w.print("%s{\"channel\":%u,\"fsctrl0\":%d,\"frames\":%lu,\"crc_ok\":%lu}", i ? "," : "", r.channel, r.fsctrl0,
            static_cast<unsigned long>(r.frames), static_cast<unsigned long>(r.crcOk));
  }
  w.print("],\"best\":");
  SurveyResult best;
  if (survey.finished() && survey.best(best)) {
    w.print("{\"channel\":%u,\"fsctrl0\":%d}", best.channel, best.fsctrl0);
  } else {
    w.print("null");
  }
  w.print("}");
}

}  // namespace

bool writeStatusJson(const StatusInput& input, char* out, size_t capacity) {
  Writer w(out, capacity);
  const LinkMonitor& monitor = *input.monitor;
  const ProxyEngine* engine = input.engine;
  const Counters& c = monitor.counters();
  const FrameDeduplicator& dedup = monitor.dedup();
  static const ProxyCounters kNoCounters{};
  const ProxyCounters& p = engine != nullptr ? engine->counters() : kNoCounters;

  const PairStatus* pair = input.pair;
  const bool wanted = pair != nullptr ? pair->wanted : (engine != nullptr && engine->proxyMode());
  const bool active = engine != nullptr && engine->proxyMode();
  const Role role = pair != nullptr ? pair->role : (engine != nullptr ? engine->role() : Role::Single);
  const char* roleName = role == Role::Bridge ? "bridge" : role == Role::Valve ? "valve" : "single";
  const char* nodeMode = !wanted ? "monitor" : !active ? "waiting" : role == Role::Single ? "proxy" : roleName;

  w.print(
      "{\"mode\":\"%s\",\"node_mode\":\"%s\",\"role\":\"%s\",\"proxy_available\":%s,\"firmware\":\"%s\","
      "\"variant\":\"%s\",\"unconfirmed\":%s,\"read_only\":%s,\"uptime_seconds\":%lu,"
      "\"memory\":{\"free\":%lu,\"min_free\":%lu},\"pair\":",
      wanted ? "proxy" : "monitor", nodeMode, roleName, engine != nullptr ? "true" : "false", input.firmware,
      input.variant, input.unconfirmed ? "true" : "false", input.readOnly ? "true" : "false",
      static_cast<unsigned long>((input.nowUs - input.startedUs) / 1000000), static_cast<unsigned long>(input.freeHeap),
      static_cast<unsigned long>(input.minFreeHeap));
  writePair(w, pair);
  w.print(",\"bridge\":");
  if (monitor.hasBridge()) {
    w.prefix(monitor.bridge());
  } else {
    w.print("null");
  }

  w.print(
      ",\"radio\":{\"found\":%s,\"version\":%u,\"channel\":%u,\"fsctrl0\":%d,\"resets\":%lu,"
      "\"overflows\":%lu},\"devices\":[",
      input.radio.found ? "true" : "false", input.radio.version, input.radio.channel, input.radio.fsctrl0,
      static_cast<unsigned long>(input.radio.resets), static_cast<unsigned long>(input.radio.overflows));
  bool first = true;
  for (size_t i = 0; i < kMaxDevices; i++) {
    const DeviceLink& device = monitor.devices()[i];
    if (device.used) {
      if (!first) {
        w.print(",");
      }
      first = false;
      writeDevice(w, device, monitor, engine, input.names, input.nowUs);
    }
  }

  float peerAnswerRssi = 0.0f;
  if (engine != nullptr && engine->peerAnswerRssi(peerAnswerRssi)) {
    w.print("],\"peer_answer_rssi_dbm\":%.1f", peerAnswerRssi);
  } else {
    w.print("],\"peer_answer_rssi_dbm\":null");
  }
  w.print(",\"windows\":{\"since_s\":%.1f,\"hour_s\":%.1f,\"day_s\":%.1f},\"trains\":{\"hour\":",
          seconds(input.nowUs - monitor.windowsSinceUs()),
          seconds(coveredUs(input.nowUs, monitor.windowsSinceUs(), kHourBucketUs, kHourBuckets)),
          seconds(coveredUs(input.nowUs, monitor.windowsSinceUs(), kDayBucketUs, kDayBuckets)));
  writeTrains(w, monitor.hourTrains());
  w.print(",\"day\":");
  writeTrains(w, monitor.dayTrains());
  w.print("},\"recent_frames\":[");
  for (size_t i = 0; i < monitor.recentCount(); i++) {
    if (i > 0) {
      w.print(",");
    }
    writeFrame(w, monitor.recent(i), input.nowUs);
  }

  w.print(
      "],\"counters\":{\"frames_rx\":%lu,\"crc_errors\":%lu,\"fcs_errors\":%lu,\"wakeups\":%lu,"
      "\"echoes\":%lu,\"verified_acks\":%lu,\"devices_not_tracked\":%lu,",
      static_cast<unsigned long>(c.framesRx), static_cast<unsigned long>(c.crcErrors),
      static_cast<unsigned long>(c.fcsErrors), static_cast<unsigned long>(c.wakeups),
      static_cast<unsigned long>(c.echoes), static_cast<unsigned long>(c.verifiedAcks),
      static_cast<unsigned long>(c.devicesNotTracked));
  w.print(
      "\"frames_tx\":%lu,\"acks_sent\":%lu,\"acks_late\":%lu,\"acks_withheld\":%lu,"
      "\"uplinks_forwarded\":%lu,\"downlinks_delivered\":%lu,\"deliveries_failed\":%lu,\"trains\":%lu,"
      "\"advertisements_forwarded\":%lu,\"duty_blocked\":%lu,\"tx_errors\":%lu,\"peer_frames_dropped\":%lu,"
      "\"errors\":0},",
      static_cast<unsigned long>(p.framesTx), static_cast<unsigned long>(p.acksSent),
      static_cast<unsigned long>(p.acksLate), static_cast<unsigned long>(p.acksWithheld),
      static_cast<unsigned long>(p.uplinksForwarded), static_cast<unsigned long>(p.downlinksDelivered),
      static_cast<unsigned long>(p.deliveriesFailed), static_cast<unsigned long>(p.trains),
      static_cast<unsigned long>(p.advertisementsForwarded), static_cast<unsigned long>(p.dutyBlocked),
      static_cast<unsigned long>(p.txErrors), static_cast<unsigned long>(p.peerFramesDropped));

  const uint64_t usedUs = engine != nullptr ? engine->duty().usedUs(input.nowUs) : 0;
  w.print(
      "\"queues\":{\"uplink\":%u,\"downlink\":%u,\"awaiting_ack\":%s},"
      "\"duty_cycle\":{\"used_s\":%.3f,\"budget_s\":%.1f,\"fraction_used\":%.4f},\"ack_latency_ms\":",
      engine != nullptr ? static_cast<unsigned>(engine->uplinksQueued()) : 0U,
      engine != nullptr ? static_cast<unsigned>(engine->downlinksQueued()) : 0U,
      engine != nullptr && engine->awaitingAck() ? "true" : "false", seconds(usedUs),
      seconds(DutyCycleLimiter::kBudgetUs),
      static_cast<double>(usedUs) / static_cast<double>(DutyCycleLimiter::kBudgetUs));
  AckLatency latency;
  if (engine != nullptr && engine->ackLatency(latency)) {
    w.print("{\"samples\":%u,\"p50\":%.2f,\"p95\":%.2f,\"max\":%.2f}", static_cast<unsigned>(latency.samples),
            latency.p50Ms, latency.p95Ms, latency.maxMs);
  } else {
    w.print("null");
  }
  w.print(",\"dedup\":{\"new\":%lu,\"retransmissions\":%lu,\"echoes\":%lu},\"survey\":",
          static_cast<unsigned long>(dedup.statsNew), static_cast<unsigned long>(dedup.statsRetransmissions),
          static_cast<unsigned long>(dedup.statsEchoes));
  if (input.survey != nullptr) {
    writeSurvey(w, *input.survey);
  } else {
    w.print("null");
  }
  w.print("}");
  return w.ok();
}

namespace {

// The first character of the value of `key`, or nullptr.
const char* findValue(const char* json, const char* key) {
  char pattern[32];
  const int length = std::snprintf(pattern, sizeof(pattern), "\"%s\"", key);
  if (length <= 0 || static_cast<size_t>(length) >= sizeof(pattern)) {
    return nullptr;
  }
  const char* position = std::strstr(json, pattern);
  if (position == nullptr) {
    return nullptr;
  }
  position += length;
  while (*position == ' ' || *position == '\t') {
    position++;
  }
  if (*position != ':') {
    return nullptr;
  }
  position++;
  while (*position == ' ' || *position == '\t') {
    position++;
  }
  return position;
}

int hexDigit(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

}  // namespace

bool jsonBoolField(const char* json, const char* key, bool& out) {
  const char* position = findValue(json, key);
  if (position == nullptr) {
    return false;
  }
  if (std::strncmp(position, "true", 4) == 0) {
    out = true;
    return true;
  }
  if (std::strncmp(position, "false", 5) == 0) {
    out = false;
    return true;
  }
  return false;
}

bool jsonStringField(const char* json, const char* key, char* out, size_t capacity) {
  const char* position = findValue(json, key);
  if (position == nullptr || *position != '"' || capacity == 0) {
    return false;
  }
  position++;
  const char* end = std::strchr(position, '"');
  if (end == nullptr || static_cast<size_t>(end - position) >= capacity) {
    return false;
  }
  const size_t length = static_cast<size_t>(end - position);
  std::memcpy(out, position, length);
  out[length] = '\0';
  return true;
}

bool parsePrefix(const char* hex, uint8_t out[kPrefixLen]) {
  if (std::strlen(hex) != kPrefixLen * 2) {
    return false;
  }
  for (size_t i = 0; i < kPrefixLen; i++) {
    const int high = hexDigit(hex[2 * i]);
    const int low = hexDigit(hex[2 * i + 1]);
    if (high < 0 || low < 0) {
      return false;
    }
    out[i] = static_cast<uint8_t>(high << 4 | low);
  }
  return true;
}

bool jsonIntField(const char* json, const char* key, int& out) {
  const char* position = findValue(json, key);
  if (position == nullptr) {
    return false;
  }
  char* end = nullptr;
  const long value = std::strtol(position, &end, 10);
  if (end == position || value < -32768 || value > 32767) {
    return false;
  }
  out = static_cast<int>(value);
  return true;
}

}  // namespace tadogado
