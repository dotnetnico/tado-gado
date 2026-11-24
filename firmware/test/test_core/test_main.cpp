#include <unity.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "dedup.h"
#include "frames.h"
#include "image_check.h"
#include "link_monitor.h"
#include "phy.h"
#include "reset_button.h"
#include "rx_assembler.h"
#include "sha256.h"
#include "status_json.h"
#include "survey.h"
#include "web_guard.h"

using namespace tadogado;

namespace {

struct FrameVector {
  const char* name;
  const char* hex;
  const char* className;
  int seq;
  int value;
  bool parses;
  int retryGroup;
};

const FrameVector kVectors[] = {
#include "frame_vectors.inc"
};

std::vector<uint8_t> fromHex(const std::string& hex) {
  std::vector<uint8_t> bytes;
  for (size_t i = 0; i + 1 < hex.size(); i += 2) {
    bytes.push_back(static_cast<uint8_t>(std::strtol(hex.substr(i, 2).c_str(), nullptr, 16)));
  }
  return bytes;
}

const FrameVector& vectorNamed(const char* name) {
  for (const FrameVector& vector : kVectors) {
    if (std::strcmp(vector.name, name) == 0) {
      return vector;
    }
  }
  TEST_FAIL_MESSAGE("no such vector");
  return kVectors[0];
}

std::vector<uint8_t> bytesOf(const char* name) { return fromHex(vectorNamed(name).hex); }

// A CC1101 FIFO whose bytes arrive over time.
class FakeBus : public RadioBus {
 public:
  uint8_t rxStatus() override {
    size_t moved = 0;
    while (!air.empty() && moved < arrivalStep) {
      fifo.push_back(air.front());
      air.erase(air.begin());
      moved++;
    }
    if (fifo.size() > 64) {
      fifo.resize(64);
      overflow = true;
    }
    return static_cast<uint8_t>(fifo.size() | (overflow ? 0x80 : 0));
  }

  void readFifo(uint8_t* out, size_t length) override {
    for (size_t i = 0; i < length; i++) {
      if (!fifo.empty()) {
        last = fifo.front();
        fifo.erase(fifo.begin());
      }
      out[i] = last;  // An empty FIFO repeats the last byte.
    }
  }

  void rearm() override {
    fifo.clear();
    overflow = false;
    rearms++;
  }

  void putOnAir(const std::vector<uint8_t>& payload, bool crcOk = true, uint8_t rssi = 0x10) {
    air.push_back(static_cast<uint8_t>(payload.size()));
    air.insert(air.end(), payload.begin(), payload.end());
    air.push_back(rssi);
    air.push_back(static_cast<uint8_t>((crcOk ? 0x80 : 0) | 0x20));
  }

  std::vector<uint8_t> air;
  std::vector<uint8_t> fifo;
  size_t arrivalStep = 1000;
  uint8_t last = 0;
  bool overflow = false;
  int rearms = 0;
};

bool pollUntilFrame(RxAssembler& assembler, RxFrame& frame, uint64_t nowUs = 1000) {
  for (int i = 0; i < 1000; i++) {
    if (assembler.poll(nowUs, frame)) {
      return true;
    }
  }
  return false;
}

RxFrame makeRx(const std::vector<uint8_t>& payload, uint64_t nowUs, bool crcOk = true) {
  RxFrame rx = {};
  std::memcpy(rx.payload, payload.data(), payload.size());
  rx.length = payload.size();
  rx.rssiDbm = -90.0f;
  rx.lqi = 30;
  rx.crcOk = crcOk;
  rx.timestampUs = nowUs;
  return rx;
}

const uint8_t kBridge[kPrefixLen] = {0xB0, 0x00, 0x00, 0x00, 0x01};
const uint8_t kFar[kPrefixLen] = {0xA0, 0x00, 0x00, 0x00, 0x02};
const uint8_t kOther[kPrefixLen] = {0xA0, 0x00, 0x00, 0x00, 0x04};

const DeviceLink* findDevice(const LinkMonitor& monitor, const uint8_t* prefix) {
  for (size_t i = 0; i < kMaxDevices; i++) {
    const DeviceLink& device = monitor.devices()[i];
    if (device.used && std::memcmp(device.prefix, prefix, kPrefixLen) == 0) {
      return &device;
    }
  }
  return nullptr;
}

}  // namespace

void setUp() {}
void tearDown() {}

// Frames ----------------------------------------------------------------------

void test_fcs_is_crc16_kermit() {
  const char* text = "123456789";
  TEST_ASSERT_EQUAL_HEX16(0x2189, macFcs(reinterpret_cast<const uint8_t*>(text), 9));
}

void test_vectors_classify_like_python() {
  for (const FrameVector& vector : kVectors) {
    const std::vector<uint8_t> frame = fromHex(vector.hex);
    TEST_ASSERT_EQUAL_STRING_MESSAGE(vector.className, frameClassName(classify(frame.data(), frame.size())),
                                     vector.name);
  }
}

void test_vectors_parse_like_python() {
  for (const FrameVector& vector : kVectors) {
    const std::vector<uint8_t> frame = fromHex(vector.hex);
    const size_t length = frame.size();
    bool parsed = false;
    int seq = 0;
    int value = 0;
    switch (classify(frame.data(), length)) {
      case FrameClass::Data:
      case FrameClass::DataPending: {
        DataFrame data;
        parsed = parseData(frame.data(), length, data);
        seq = parsed ? data.seq : 0;
        break;
      }
      case FrameClass::Broadcast: {
        BroadcastFrame broadcast;
        parsed = parseBroadcast(frame.data(), length, broadcast);
        seq = parsed ? broadcast.seq : 0;
        break;
      }
      case FrameClass::Wakeup: {
        WakeUp wake;
        parsed = parseWakeup(frame.data(), length, wake);
        seq = parsed ? wake.seq : 0;
        value = parsed ? wake.rendezvous : 0;
        break;
      }
      case FrameClass::EnhAck: {
        EnhAck ack;
        parsed = parseEnhAck(frame.data(), length, ack);
        seq = parsed ? ack.seq : 0;
        value = parsed ? ack.phase : 0;
        break;
      }
      default:
        break;
    }
    TEST_ASSERT_EQUAL_MESSAGE(vector.parses, parsed, vector.name);
    if (parsed) {
      TEST_ASSERT_EQUAL_MESSAGE(vector.seq, seq, vector.name);
      TEST_ASSERT_EQUAL_MESSAGE(vector.value, value, vector.name);
    }
  }
}

void test_retransmission_hash_groups_retries() {
  const std::vector<uint8_t> first = bytesOf("data");
  const std::vector<uint8_t> retry = bytesOf("data_retry");
  const std::vector<uint8_t> other = bytesOf("data_other");
  TEST_ASSERT_TRUE(first != retry);
  TEST_ASSERT_TRUE(retransmissionHash(first.data(), first.size()) == retransmissionHash(retry.data(), retry.size()));
  TEST_ASSERT_TRUE(retransmissionHash(first.data(), first.size()) != retransmissionHash(other.data(), other.size()));
}

void test_airtime_of_a_wake_frame() { TEST_ASSERT_UINT32_WITHIN(20, 3362, airtimeUs(12)); }

uint8_t phyRegister(uint8_t address) {
  for (const PhyRegister& reg : kPhyRegisters) {
    if (reg.address == address) {
      return reg.value;
    }
  }
  TEST_FAIL_MESSAGE("register missing");
  return 0;
}

// The derived PHY values of freedo's PROTOCOL.md, from the registers and a 26 MHz crystal.
void test_phy_matches_the_tado_air_interface() {
  constexpr double kXtalHz = 26e6;
  const double base = kXtalHz / 65536.0 * ((phyRegister(0x0D) << 16) | (phyRegister(0x0E) << 8) | phyRegister(0x0F));
  const uint8_t mdmcfg1 = phyRegister(0x13);
  const double spacing = kXtalHz / 262144.0 * (256 + phyRegister(0x14)) * (1 << (mdmcfg1 & 0x03));
  const double carrier = base + spacing * phyRegister(kRegChannr);
  const uint8_t mdmcfg4 = phyRegister(0x10);
  const double baud = kXtalHz / 268435456.0 * (256 + phyRegister(0x11)) * (1 << (mdmcfg4 & 0x0F));
  const double bandwidth = kXtalHz / (8.0 * (4 + ((mdmcfg4 >> 4) & 0x03)) * (1 << (mdmcfg4 >> 6)));
  const uint8_t deviatn = phyRegister(0x15);
  const double deviation = kXtalHz / 131072.0 * (8 + (deviatn & 0x07)) * (1 << ((deviatn >> 4) & 0x07));

  TEST_ASSERT_DOUBLE_WITHIN(1e3, 863.125e6, base);
  TEST_ASSERT_DOUBLE_WITHIN(10, 199951, spacing);
  TEST_ASSERT_DOUBLE_WITHIN(1e3, 868.324e6, carrier);
  TEST_ASSERT_DOUBLE_WITHIN(10, 49990, baud);
  TEST_ASSERT_DOUBLE_WITHIN(1e3, 135e3, bandwidth);
  TEST_ASSERT_DOUBLE_WITHIN(10, 25390, deviation);
  TEST_ASSERT_EQUAL_HEX8(0x13, phyRegister(0x12));  // GFSK, 30/32 sync bits
  TEST_ASSERT_EQUAL_HEX8(0x22, mdmcfg1);            // 4-byte preamble
  TEST_ASSERT_EQUAL_HEX8(0xD3, phyRegister(0x04));
  TEST_ASSERT_EQUAL_HEX8(0x91, phyRegister(0x05));
  TEST_ASSERT_EQUAL_HEX8(0x05, phyRegister(0x08));  // hardware CRC on
  TEST_ASSERT_EQUAL_HEX8(0x08, phyRegister(0x18));  // manual calibration
  TEST_ASSERT_EQUAL(kDefaultChannel, phyRegister(kRegChannr));
}

// Dedup -----------------------------------------------------------------------

void test_dedup_sorts_new_echo_and_retry() {
  FrameDeduplicator dedup;
  const std::vector<uint8_t> first = bytesOf("data");
  const std::vector<uint8_t> retry = bytesOf("data_retry");
  TEST_ASSERT_TRUE(dedup.classify(first.data(), first.size(), 1000) == Verdict::New);
  TEST_ASSERT_TRUE(dedup.classify(first.data(), first.size(), 2000) == Verdict::Echo);
  TEST_ASSERT_TRUE(dedup.classify(retry.data(), retry.size(), 3000) == Verdict::Retransmission);
}

void test_dedup_window_expires() {
  FrameDeduplicator dedup;
  const std::vector<uint8_t> first = bytesOf("data");
  const std::vector<uint8_t> retry = bytesOf("data_retry");
  dedup.classify(first.data(), first.size(), 1000);
  TEST_ASSERT_TRUE(dedup.classify(retry.data(), retry.size(), 11000000) == Verdict::New);
}

void test_dedup_table_survives_more_frames_than_it_holds() {
  FrameDeduplicator dedup;
  std::vector<uint8_t> frame = bytesOf("data");
  for (int i = 0; i < 1000; i++) {
    frame[16] = static_cast<uint8_t>(i);  // Ciphertext outside the frame counter.
    frame[17] = static_cast<uint8_t>(i >> 8);
    dedup.classify(frame.data(), frame.size(), 1000 + static_cast<uint64_t>(i));
  }
  TEST_ASSERT_EQUAL_UINT32(1000, dedup.statsNew);
}

// Receive ---------------------------------------------------------------------

void test_frame_arriving_byte_by_byte_is_whole() {
  FakeBus bus;
  RxAssembler assembler(bus);
  const std::vector<uint8_t> payload = bytesOf("data");
  bus.arrivalStep = 1;
  bus.putOnAir(payload);
  RxFrame frame;
  TEST_ASSERT_TRUE(pollUntilFrame(assembler, frame));
  TEST_ASSERT_EQUAL_size_t(payload.size(), frame.length);
  TEST_ASSERT_EQUAL_MEMORY(payload.data(), frame.payload, payload.size());
  TEST_ASSERT_TRUE(frame.crcOk);
}

void test_frame_longer_than_the_fifo_is_whole() {
  FakeBus bus;
  RxAssembler assembler(bus);
  std::vector<uint8_t> payload(120);
  for (size_t i = 0; i < payload.size(); i++) {
    payload[i] = static_cast<uint8_t>(i * 3);
  }
  bus.arrivalStep = 8;
  bus.putOnAir(payload);
  RxFrame frame;
  TEST_ASSERT_TRUE(pollUntilFrame(assembler, frame));
  TEST_ASSERT_EQUAL_MEMORY(payload.data(), frame.payload, payload.size());
}

void test_fifo_is_never_emptied_in_mid_packet() {
  FakeBus bus;
  RxAssembler assembler(bus);
  bus.arrivalStep = 3;
  bus.putOnAir(std::vector<uint8_t>(30, 0x55));
  RxFrame frame;
  while (!bus.air.empty()) {
    assembler.poll(1000, frame);
    TEST_ASSERT_TRUE(!bus.fifo.empty() || bus.air.empty());
  }
}

void test_two_frames_in_a_row() {
  FakeBus bus;
  RxAssembler assembler(bus);
  bus.putOnAir({1, 2, 3});
  bus.putOnAir({4, 5});
  RxFrame frame;
  TEST_ASSERT_TRUE(pollUntilFrame(assembler, frame));
  TEST_ASSERT_EQUAL_size_t(3, frame.length);
  TEST_ASSERT_TRUE(pollUntilFrame(assembler, frame));
  TEST_ASSERT_EQUAL_size_t(2, frame.length);
  TEST_ASSERT_EQUAL_UINT8(5, frame.payload[1]);
}

void test_status_bytes_are_decoded() {
  FakeBus bus;
  RxAssembler assembler(bus);
  bus.putOnAir({1, 2, 3}, false, 0xC0);
  RxFrame frame;
  TEST_ASSERT_TRUE(pollUntilFrame(assembler, frame));
  TEST_ASSERT_FALSE(frame.crcOk);
  TEST_ASSERT_EQUAL_FLOAT(-106.0f, frame.rssiDbm);
  TEST_ASSERT_EQUAL_UINT8(0x20, frame.lqi);
}

void test_impossible_length_flushes() {
  for (uint8_t length : {uint8_t{0}, uint8_t{128}, uint8_t{255}}) {
    FakeBus bus;
    RxAssembler assembler(bus);
    bus.air = {length, 1, 2};
    RxFrame frame;
    TEST_ASSERT_FALSE(assembler.poll(1000, frame));
    TEST_ASSERT_EQUAL_UINT32(1, assembler.badLengths);
    TEST_ASSERT_EQUAL(1, bus.rearms);
  }
}

void test_overflow_recovers() {
  FakeBus bus;
  RxAssembler assembler(bus);
  bus.air.assign(100, 0);
  RxFrame frame;
  TEST_ASSERT_FALSE(assembler.poll(1000, frame));
  TEST_ASSERT_EQUAL_UINT32(1, assembler.overflows);
  TEST_ASSERT_EQUAL(1, bus.rearms);
}

void test_stalled_frame_is_abandoned() {
  FakeBus bus;
  RxAssembler assembler(bus);
  bus.air = {20, 1, 2, 3, 4};
  RxFrame frame;
  assembler.poll(1000, frame);
  TEST_ASSERT_TRUE(assembler.busy());
  bus.fifo.clear();
  assembler.poll(1000 + 31000, frame);
  TEST_ASSERT_FALSE(assembler.busy());
  TEST_ASSERT_EQUAL_UINT32(1, assembler.stalls);
}

// Monitor ---------------------------------------------------------------------

std::vector<uint8_t> dataFrame(uint8_t seq, const uint8_t* dstPrefix, const uint8_t* srcPrefix, uint8_t tag = 0) {
  std::vector<uint8_t> frame = {0x69, 0xEC, seq};
  frame.insert(frame.end(), dstPrefix, dstPrefix + kPrefixLen);
  frame.insert(frame.end(), {0xC5, 0x1B, 0x00});
  frame.insert(frame.end(), srcPrefix, srcPrefix + kPrefixLen);
  frame.insert(frame.end(), 20, tag);
  return frame;
}

std::vector<uint8_t> enhAck(uint8_t seq, const uint8_t* dstPrefix, const uint8_t* srcPrefix) {
  std::vector<uint8_t> frame = {0x42, 0xEE, seq};
  frame.insert(frame.end(), dstPrefix, dstPrefix + kPrefixLen);
  frame.insert(frame.end(), {0xC5, 0x1B, 0x00});
  frame.insert(frame.end(), srcPrefix, srcPrefix + kPrefixLen);
  frame.insert(frame.end(), {0xC5, 0x1B, 0x00, 0x04, 0x0D, 0x10, 0x00, 0x35, 0x0C, 0x80, 0x3F});
  const uint16_t fcs = macFcs(frame.data(), frame.size());
  frame.push_back(static_cast<uint8_t>(fcs));
  frame.push_back(static_cast<uint8_t>(fcs >> 8));
  return frame;
}

std::vector<uint8_t> wakeup(uint8_t seq) {
  return {0x25, seq, 0xCD, 0xAB, 0x00, 0xA0, 0x82, 0x0E, 0x00, 0x00, 0x80, 0x3F};
}

void test_bridge_is_learned_from_two_trains() {
  LinkMonitor monitor;
  monitor.onFrame(makeRx(wakeup(7), 1000000));
  monitor.onFrame(makeRx(dataFrame(7, kFar, kBridge), 1020000));
  TEST_ASSERT_FALSE(monitor.hasBridge());
  monitor.onFrame(makeRx(wakeup(8), 3000000));
  monitor.onFrame(makeRx(dataFrame(8, kFar, kBridge), 3020000));
  TEST_ASSERT_TRUE(monitor.hasBridge());
  TEST_ASSERT_EQUAL_MEMORY(kBridge, monitor.bridge(), kPrefixLen);
}

void test_a_forged_train_from_another_sender_does_not_name_the_bridge() {
  LinkMonitor monitor;
  monitor.onFrame(makeRx(wakeup(7), 1000000));
  monitor.onFrame(makeRx(dataFrame(7, kFar, kBridge), 1020000));
  monitor.onFrame(makeRx(wakeup(8), 3000000));
  monitor.onFrame(makeRx(dataFrame(8, kBridge, kFar), 3020000));  // Another sender.
  TEST_ASSERT_FALSE(monitor.hasBridge());
}

void test_data_without_a_train_teaches_nothing() {
  LinkMonitor monitor;
  monitor.onFrame(makeRx(dataFrame(7, kBridge, kFar), 1000000));
  TEST_ASSERT_FALSE(monitor.hasBridge());
}

void test_acked_uplink_counts_as_good() {
  LinkMonitor monitor;
  monitor.setBridge(kBridge);
  monitor.onFrame(makeRx(dataFrame(1, kBridge, kFar), 1000000));
  monitor.onFrame(makeRx(enhAck(1, kFar, kBridge), 1006000));
  monitor.tick(1100000);
  const DeviceLink* far = findDevice(monitor, kFar);
  TEST_ASSERT_NOT_NULL(far);
  TEST_ASSERT_EQUAL_UINT32(1, far->uplinks);
  TEST_ASSERT_EQUAL_UINT32(1, far->uplinksAcked);
  TEST_ASSERT_EQUAL_UINT32(0, far->uplinksUnacked);
  TEST_ASSERT_EQUAL_UINT32(1, monitor.counters().verifiedAcks);
}

void test_unanswered_uplink_counts_as_bad() {
  LinkMonitor monitor;
  monitor.setBridge(kBridge);
  monitor.onFrame(makeRx(dataFrame(1, kBridge, kFar), 1000000));
  monitor.tick(1020000);
  TEST_ASSERT_EQUAL_UINT32(0, findDevice(monitor, kFar)->uplinksUnacked);
  monitor.tick(1030000);
  TEST_ASSERT_EQUAL_UINT32(1, findDevice(monitor, kFar)->uplinksUnacked);
}

void test_downlink_ack_is_tracked() {
  LinkMonitor monitor;
  monitor.setBridge(kBridge);
  monitor.onFrame(makeRx(dataFrame(2, kFar, kBridge), 1000000));
  monitor.onFrame(makeRx(enhAck(2, kBridge, kFar), 1006000));
  TEST_ASSERT_EQUAL_UINT32(1, findDevice(monitor, kFar)->downlinksAcked);
}

void test_crc_failure_is_only_counted() {
  LinkMonitor monitor;
  monitor.onFrame(makeRx(dataFrame(1, kBridge, kFar), 1000000, false));
  TEST_ASSERT_EQUAL_UINT32(1, monitor.counters().crcErrors);
  TEST_ASSERT_EQUAL_size_t(0, monitor.recentCount());
}

void test_bad_fcs_is_counted_and_not_verified() {
  LinkMonitor monitor;
  std::vector<uint8_t> ack = enhAck(1, kFar, kBridge);
  ack[4] ^= 0xFF;
  monitor.onFrame(makeRx(ack, 1000000));
  TEST_ASSERT_EQUAL_UINT32(1, monitor.counters().fcsErrors);
  TEST_ASSERT_EQUAL_UINT32(0, monitor.counters().verifiedAcks);
}

void test_device_table_full_is_counted_not_fatal() {
  LinkMonitor monitor;
  monitor.setBridge(kBridge);
  for (int i = 0; i < 60; i++) {
    const uint8_t prefix[kPrefixLen] = {0xA1, 0, 0, 0, static_cast<uint8_t>(i)};
    monitor.onFrame(makeRx(dataFrame(static_cast<uint8_t>(i), kBridge, prefix), 1000000));
  }
  TEST_ASSERT_GREATER_THAN_UINT32(0, monitor.counters().devicesNotTracked);
}

void test_recent_frames_are_newest_first_and_bounded() {
  LinkMonitor monitor;
  monitor.setBridge(kBridge);
  for (int i = 0; i < 40; i++) {
    monitor.onFrame(makeRx(dataFrame(static_cast<uint8_t>(i), kBridge, kFar), 1000000 + i));
  }
  TEST_ASSERT_EQUAL_size_t(kRecentFrames, monitor.recentCount());
  TEST_ASSERT_EQUAL_UINT8(39, monitor.recent(0).seq);
  TEST_ASSERT_EQUAL_UINT8(15, monitor.recent(kRecentFrames - 1).seq);
}

// Survey ----------------------------------------------------------------------

void test_trim_adds_the_median_offset() {
  const int8_t readings[] = {-13, -13, -12, -14, -13};
  int8_t out = 0;
  TEST_ASSERT_TRUE(suggestFsctrl0(0x40, readings, 5, out));
  TEST_ASSERT_EQUAL_INT8(0x33, out);
}

void test_trim_ignores_clipped_readings() {
  const int8_t readings[] = {-21, -21, 5, 5, 5};
  int8_t out = 0;
  TEST_ASSERT_TRUE(suggestFsctrl0(0, readings, 5, out));
  TEST_ASSERT_EQUAL_INT8(5, out);
}

void test_trim_rounds_like_python() {
  const int8_t readings[] = {1, 4};  // Median 2.5, which Python rounds to 2.
  int8_t out = 0;
  TEST_ASSERT_TRUE(suggestFsctrl0(0, readings, 2, out));
  TEST_ASSERT_EQUAL_INT8(2, out);
  const int8_t odd[] = {3, 4};  // Median 3.5, which Python rounds to 4.
  TEST_ASSERT_TRUE(suggestFsctrl0(0, odd, 2, out));
  TEST_ASSERT_EQUAL_INT8(4, out);
}

void test_no_usable_reading_gives_no_trim() {
  const int8_t readings[] = {-21, 21};
  int8_t out = 0;
  TEST_ASSERT_FALSE(suggestFsctrl0(0, readings, 2, out));
  TEST_ASSERT_FALSE(suggestFsctrl0(0, readings, 0, out));
}

void test_survey_visits_every_step_and_finds_the_best() {
  Survey survey;
  survey.start(0, 1000);
  TEST_ASSERT_TRUE(survey.tick(0));
  TEST_ASSERT_EQUAL_UINT8(26, survey.current().channel);
  size_t retunes = 1;
  uint64_t now = 0;
  while (survey.running()) {
    if (survey.current().channel == 25 && survey.current().fsctrl0 == 32) {
      survey.onFrame(true, 2);
    }
    now += 1000;
    if (survey.tick(now)) {
      retunes++;
    }
  }
  TEST_ASSERT_EQUAL_size_t(Survey::kResults, retunes);
  TEST_ASSERT_TRUE(survey.finished());
  SurveyResult best;
  TEST_ASSERT_TRUE(survey.best(best));
  TEST_ASSERT_EQUAL_UINT8(25, best.channel);
  TEST_ASSERT_EQUAL_INT8(34, best.fsctrl0);
}

void test_silent_survey_has_no_best() {
  Survey survey;
  survey.start(0, 10);
  uint64_t now = 0;
  while (survey.running()) {
    survey.tick(now += 10);
  }
  SurveyResult best;
  TEST_ASSERT_FALSE(survey.best(best));
}

// Status ----------------------------------------------------------------------

std::string statusOf(const LinkMonitor& monitor, const Survey* survey, char* buffer, size_t capacity) {
  StatusInput input = {};
  input.monitor = &monitor;
  input.survey = survey;
  input.radio = {true, 0x14, 26, -13, 0, 0};
  input.firmware = "1.2.3";
  input.variant = "esp32s3octal";
  input.readOnly = true;
  input.nowUs = 5000000;
  TEST_ASSERT_TRUE(writeStatusJson(input, buffer, capacity));
  return buffer;
}

void test_status_has_every_field_the_dashboard_reads() {
  static char buffer[24000];
  LinkMonitor monitor;
  monitor.setBridge(kBridge);
  monitor.onFrame(makeRx(dataFrame(1, kBridge, kFar), 1000000));
  monitor.onFrame(makeRx(enhAck(1, kFar, kBridge), 1006000));
  const std::string json = statusOf(monitor, nullptr, buffer, sizeof(buffer));
  for (const char* key : {"\"mode\":\"monitor\"",
                          "\"role\"",
                          "\"uptime_seconds\":5",
                          "\"read_only\":true",
                          "\"variant\":\"esp32s3octal\"",
                          "\"unconfirmed\":false",
                          "\"bridge\":\"b000000001\"",
                          "\"proxy_available\":false",
                          "\"device\":\"a000000002\"",
                          "\"rssi_dbm\"",
                          "\"lqi\"",
                          "\"seconds_since_seen\"",
                          "\"uplink_ack_rate\":1.000",
                          "\"downlink_ack_rate\":null",
                          "\"retransmissions\"",
                          "\"proxied_uplinks\"",
                          "\"proxied_downlinks\"",
                          "\"delivery_failures\"",
                          "\"airtime_s\":0.00",
                          "\"kind\":\"bridge\"",
                          "\"recent_frames\"",
                          "\"age_s\"",
                          "\"class\":\"enh-ack\"",
                          "\"frames_rx\":2",
                          "\"crc_errors\":0",
                          "\"verified_acks\":1",
                          "\"uplinks_forwarded\"",
                          "\"downlinks_delivered\"",
                          "\"deliveries_failed\"",
                          "\"fraction_used\"",
                          "\"ack_latency_ms\":null"}) {
    TEST_ASSERT_NOT_NULL_MESSAGE(std::strstr(json.c_str(), key), key);
  }
}

void test_worst_case_status_fits_the_firmware_buffer() {
  static char buffer[kStatusJsonCapacity];
  LinkMonitor monitor;
  monitor.setBridge(kBridge);
  for (int i = 0; i < 60; i++) {
    const uint8_t prefix[kPrefixLen] = {0xA1, 0, 0, 0, static_cast<uint8_t>(i)};
    monitor.onFrame(makeRx(dataFrame(static_cast<uint8_t>(i), kBridge, prefix), 1000000));
    const DeviceLink* known = monitor.findLink(prefix);
    if (known != nullptr) {
      DeviceLink& link = *monitor.mutableLink(prefix);
      for (uint32_t* count : {&link.frames, &link.uplinks, &link.downlinks, &link.retransmissions, &link.proxiedUplinks,
                              &link.proxiedDownlinks, &link.deliveryFailures}) {
        *count = 4000000000;
      }
      link.airtime.record(DutyCycleLimiter::kBudgetUs, 1000000);
    }
  }
  Survey survey;
  survey.start(0, 1);
  uint64_t now = 0;
  while (survey.running()) {
    survey.onFrame(true, 0);
    survey.tick(now += 2);
  }
  statusOf(monitor, &survey, buffer, sizeof(buffer));
}

void test_status_reports_a_buffer_that_is_too_small() {
  char buffer[64];
  LinkMonitor monitor;
  StatusInput input = {};
  input.monitor = &monitor;
  input.firmware = "1.2.3";
  input.variant = "esp32s3octal";
  TEST_ASSERT_FALSE(writeStatusJson(input, buffer, sizeof(buffer)));
  TEST_ASSERT_LESS_THAN_size_t(sizeof(buffer), std::strlen(buffer) + 1);
}

void test_json_int_field() {
  int value = 0;
  TEST_ASSERT_TRUE(jsonIntField("{\"channel\": 26, \"fsctrl0\":-13}", "fsctrl0", value));
  TEST_ASSERT_EQUAL(-13, value);
  TEST_ASSERT_TRUE(jsonIntField("{\"channel\": 26, \"fsctrl0\":-13}", "channel", value));
  TEST_ASSERT_EQUAL(26, value);
  TEST_ASSERT_FALSE(jsonIntField("{\"channel\": \"a\"}", "channel", value));
  TEST_ASSERT_FALSE(jsonIntField("{}", "channel", value));
  TEST_ASSERT_FALSE(jsonIntField("{\"channel\" 26}", "channel", value));
}

// Builders --------------------------------------------------------------------

void test_builders_match_the_python_vectors() {
  const std::vector<uint8_t> wake = bytesOf("wakeup");
  uint8_t builtWake[kWakeupLen];
  buildWakeup(0x12, wake.data() + 4, 248, builtWake);
  TEST_ASSERT_EQUAL_MEMORY(wake.data(), builtWake, kWakeupLen);

  const std::vector<uint8_t> ack = bytesOf("enh_ack");
  uint8_t builtAck[kEnhAckLen];
  buildEnhAck(7, ack.data() + 3, ack.data() + 11, 1234, builtAck);
  TEST_ASSERT_EQUAL_MEMORY(ack.data(), builtAck, kEnhAckLen);
}

// Duty cycle and anchors ------------------------------------------------------

void test_duty_budget_blocks_and_returns() {
  DutyCycleLimiter duty;
  const uint64_t minute = 60000000;
  duty.record(DutyCycleLimiter::kBudgetUs, 10 * minute);
  TEST_ASSERT_FALSE(duty.allows(1, 10 * minute));
  TEST_ASSERT_FALSE(duty.allows(1, 69 * minute));
  TEST_ASSERT_TRUE(duty.allows(1, 72 * minute));
}

void test_airtime_of_an_hour_is_dropped_a_bucket_late() {
  HourlyAirtime<300> airtime;
  const uint64_t minute = 60000000;
  airtime.record(1000, 4 * minute);
  airtime.record(500, 6 * minute);
  TEST_ASSERT_EQUAL_UINT64(1500, airtime.usedUs(64 * minute));
  TEST_ASSERT_EQUAL_UINT64(500, airtime.usedUs(65 * minute));
  TEST_ASSERT_EQUAL_UINT64(0, airtime.usedUs(70 * minute));
  airtime.record(7, 69 * minute);  // Reuses the bucket of minute 4.
  TEST_ASSERT_EQUAL_UINT64(507, airtime.usedUs(69 * minute));
}

// The ACKs of a pair are ambiguous only for proxied devices, and only while it proxies.
void test_acks_of_other_devices_are_still_measured_in_a_pair() {
  LinkMonitor monitor;
  monitor.setBridge(kBridge);
  monitor.setProxied(kFar, true);
  uint8_t dst[kEuiLen] = {};
  uint8_t src[kEuiLen] = {};
  uint8_t ack[kEnhAckLen];
  const auto hear = [&](const uint8_t* from, const uint8_t* to) {
    std::memcpy(src, from, kPrefixLen);
    std::memcpy(dst, to, kPrefixLen);
    buildEnhAck(1, dst, src, 0, ack);
    monitor.onFrame(makeRx(std::vector<uint8_t>(ack, ack + kEnhAckLen), 1000000));
  };
  monitor.setProxyActive(true, true);
  hear(kFar, kBridge);
  hear(kBridge, kFar);
  TEST_ASSERT_EQUAL(0, monitor.counters().verifiedAcks);
  TEST_ASSERT_EQUAL(2, monitor.recentCount());
  const uint8_t other[kPrefixLen] = {0xA0, 0, 0, 0, 9};
  hear(other, kBridge);
  TEST_ASSERT_EQUAL(1, monitor.counters().verifiedAcks);
  monitor.setProxyActive(true, false);
  hear(kFar, kBridge);
  monitor.setProxyActive(false, true);
  hear(kFar, kBridge);
  TEST_ASSERT_EQUAL(3, monitor.counters().verifiedAcks);
  TEST_ASSERT_EQUAL(2, monitor.findLink(kFar)->frames);
}

void test_handed_over_frames_are_known_for_as_long_as_the_other_node_tries() {
  LinkMonitor monitor;
  const uint8_t frame[4] = {1, 2, 3, 4};
  TEST_ASSERT_FALSE(monitor.sentByOtherNode(frame, sizeof(frame), 1));
  monitor.noteHandedOver(frame, sizeof(frame), 1000000);
  TEST_ASSERT_TRUE(monitor.sentByOtherNode(frame, sizeof(frame), 200000000));
  TEST_ASSERT_FALSE(monitor.sentByOtherNode(frame, sizeof(frame) - 1, 200000000));
}

// The start of an application image, then the text.
std::string imageOf(const std::string& text) {
  std::string image(ImageCheck::kHeaderBytes, '\0');
  image[0] = '\xE9';
  image[12] = 9;
  image.replace(32, 4, "\x32\x54\xCD\xAB");
  return image + text;
}

const char* verdictOf(const std::string& image, std::string* version = nullptr, size_t piece = 4096) {
  ImageCheck check("esp32s3octal");
  for (size_t i = 0; i < image.size(); i += piece) {
    const std::string part = image.substr(i, piece);
    check.feed(reinterpret_cast<const uint8_t*>(part.data()), part.size());
  }
  if (version != nullptr) {
    *version = check.version();
  }
  return check.error();
}

const std::string kMarker = std::string("tado-gado-image:esp32s3octal:0.8.0") + '\0';

void test_image_for_this_board_is_accepted_in_pieces_of_any_size() {
  const std::string image = imageOf("code" + kMarker + "more code");
  for (size_t piece = 1; piece <= image.size(); piece++) {
    std::string version;
    TEST_ASSERT_NULL(verdictOf(image, &version, piece));
    TEST_ASSERT_EQUAL_STRING("0.8.0", version.c_str());
  }
}

void test_marker_is_found_among_text_that_looks_like_it() {
  const std::string nul(1, '\0');
  for (const std::string& before : {"tado-gado-image:" + nul, "tado-gado-image:([a-z" + nul,
                                    "tado-gado-tado-gado-image" + nul, std::string("tado-gado-image:esp32s3:"),
                                    "tado-gado-image:esp32s3octal:" + nul, "tado-gado-image:esp32s3octal:9.9.9x" + nul,
                                    "tado-gado-image::1.0" + nul, "tado-gado-image:abcdefghijklmnop:1.0" + nul}) {
    std::string version;
    TEST_ASSERT_NULL(verdictOf(imageOf(before + kMarker), &version));
    TEST_ASSERT_EQUAL_STRING("0.8.0", version.c_str());
    TEST_ASSERT_NOT_NULL(verdictOf(imageOf(before)));
  }
}

// A marker of the same shape under another name is some other firmware.
void test_image_with_another_projects_marker_is_refused() {
  const std::string other = std::string("other-image:esp32s3octal:0.11.0") + '\0';
  std::string version;
  TEST_ASSERT_NULL(verdictOf(imageOf("code" + kMarker + other), &version));
  TEST_ASSERT_EQUAL_STRING("0.8.0", version.c_str());
  TEST_ASSERT_NOT_NULL(std::strstr(verdictOf(imageOf("code" + other)), "not a tado-gado image"));
}

void test_wrong_images_are_refused() {
  TEST_ASSERT_NOT_NULL(verdictOf(""));
  TEST_ASSERT_NOT_NULL(verdictOf(imageOf("").substr(0, ImageCheck::kHeaderBytes - 1)));
  TEST_ASSERT_NOT_NULL(std::strstr(verdictOf(imageOf("code")), "not a tado-gado image"));
  TEST_ASSERT_NOT_NULL(
      std::strstr(verdictOf(imageOf(std::string("tado-gado-image:esp32s3:0.8.0") + '\0')), "other kind"));
  std::string image = imageOf(kMarker);
  image[0] = 'P';
  TEST_ASSERT_NOT_NULL(std::strstr(verdictOf(image), "not a firmware image"));
  image = imageOf(kMarker);
  image[12] = 5;
  TEST_ASSERT_NOT_NULL(std::strstr(verdictOf(image), "another chip"));
  image = imageOf(kMarker);
  image[13] = 1;
  TEST_ASSERT_NOT_NULL(std::strstr(verdictOf(image), "another chip"));
  // The factory image starts with the bootloader, and holds the marker further on.
  image = imageOf(kMarker);
  image[35] = 0;
  TEST_ASSERT_NOT_NULL(std::strstr(verdictOf(image), "factory image"));
}

void test_slot_is_centred_on_a_sample_and_in_the_future() {
  AnchorTable anchors;
  anchors.update(kFar, 100500000, 625);  // 200 ms after a sample.
  Slot slot;
  TEST_ASSERT_TRUE(anchors.nextSlot(kFar, 101000000, slot));
  TEST_ASSERT_TRUE(slot.startUs > 101000000);
  TEST_ASSERT_EQUAL_UINT64(300000, ((slot.startUs + slot.endUs) / 2) % 1000000);
}

void test_large_phase_wraps_on_the_period() {
  AnchorTable anchors;
  anchors.update(kFar, 100000000, 3125 * 7 + 625);
  Slot slot;
  TEST_ASSERT_TRUE(anchors.nextSlot(kFar, 100100000, slot));
  TEST_ASSERT_EQUAL_UINT64(800000, ((slot.startUs + slot.endUs) / 2) % 1000000);
}

void test_old_or_missed_anchor_gives_no_slot() {
  AnchorTable anchors;
  Slot slot;
  TEST_ASSERT_FALSE(anchors.nextSlot(kFar, 1000, slot));
  anchors.update(kFar, 0, 0);
  TEST_ASSERT_FALSE(anchors.nextSlot(kFar, 20000ULL * 1000000ULL, slot));
  for (int i = 0; i < 4; i++) {
    TEST_ASSERT_TRUE(anchors.contains(kFar));
    anchors.miss(kFar);
  }
  TEST_ASSERT_FALSE(anchors.contains(kFar));
}

void test_anchor_phase_round_trip() {
  AnchorTable anchors;
  anchors.update(kFar, 100300000, 0);
  uint16_t phase = 0;
  TEST_ASSERT_TRUE(anchors.phase(kFar, 100500000, phase));
  TEST_ASSERT_UINT16_WITHIN(1, 625, phase);
}

// Proxy -----------------------------------------------------------------------

class FakePort : public RadioPort {
 public:
  bool send(const uint8_t* frame, size_t length) override {
    sent.emplace_back(frame, frame + length);
    clock += airtimeUs(length);
    return true;
  }
  void sleepUntil(uint64_t timeUs) override { clock = timeUs > clock ? timeUs : clock; }
  uint64_t nowUs() override { return clock; }

  std::vector<std::vector<uint8_t>> sent;
  uint64_t clock = 10000000;
};

void runFor(ProxyEngine& engine, FakePort& port, uint64_t durationUs) {
  const uint64_t end = port.clock + durationUs;
  while (port.clock < end) {
    engine.tick();
    port.clock += 500;
  }
}

void test_engine_starts_in_monitor_and_never_transmits() {
  FakePort port;
  LinkMonitor monitor;
  monitor.setBridge(kBridge);
  monitor.setProxied(kFar, true);
  ProxyEngine engine(port, monitor);
  TEST_ASSERT_FALSE(engine.proxyMode());
  engine.onFrame(makeRx(dataFrame(1, kBridge, kFar), port.clock));
  engine.onFrame(makeRx(dataFrame(2, kFar, kBridge), port.clock));
  runFor(engine, port, 5000000);
  TEST_ASSERT_EQUAL_size_t(0, port.sent.size());
  TEST_ASSERT_EQUAL_UINT32(0, engine.counters().framesTx);
}

void test_proxy_acks_then_forwards_byte_for_byte() {
  FakePort port;
  LinkMonitor monitor;
  monitor.setBridge(kBridge);
  monitor.setProxied(kFar, true);
  ProxyEngine engine(port, monitor);
  engine.setProxyMode(true);
  const std::vector<uint8_t> frame = dataFrame(42, kBridge, kFar, 7);
  engine.onFrame(makeRx(frame, port.clock));
  runFor(engine, port, 50000);
  TEST_ASSERT_EQUAL_size_t(2, port.sent.size());
  EnhAck ack;
  TEST_ASSERT_TRUE(parseEnhAck(port.sent[0].data(), port.sent[0].size(), ack));
  TEST_ASSERT_EQUAL_UINT8(42, ack.seq);
  TEST_ASSERT_EQUAL_MEMORY(kFar, ack.dstEui, kPrefixLen);
  TEST_ASSERT_EQUAL_MEMORY(kBridge, ack.srcEui, kPrefixLen);
  TEST_ASSERT_TRUE(port.sent[1] == frame);
}

void test_unnamed_device_is_left_alone() {
  FakePort port;
  LinkMonitor monitor;
  monitor.setBridge(kBridge);
  ProxyEngine engine(port, monitor);
  engine.setProxyMode(true);
  engine.onFrame(makeRx(dataFrame(1, kBridge, kFar), port.clock));
  runFor(engine, port, 1000000);
  TEST_ASSERT_EQUAL_size_t(0, port.sent.size());
}

void test_late_ack_is_dropped_and_counted() {
  FakePort port;
  LinkMonitor monitor;
  monitor.setBridge(kBridge);
  monitor.setProxied(kFar, true);
  ProxyEngine engine(port, monitor);
  engine.setProxyMode(true);
  engine.onFrame(makeRx(dataFrame(1, kBridge, kFar), port.clock));
  port.clock += 100000;
  engine.tick();
  TEST_ASSERT_EQUAL_UINT32(1, engine.counters().acksLate);
  TEST_ASSERT_EQUAL_UINT32(0, engine.counters().acksSent);
}

void test_forward_is_retried_then_abandoned_and_reported() {
  FakePort port;
  LinkMonitor monitor;
  monitor.setBridge(kBridge);
  monitor.setProxied(kFar, true);
  ProxyEngine engine(port, monitor);
  engine.setProxyMode(true);
  const std::vector<uint8_t> frame = dataFrame(1, kBridge, kFar);
  engine.onFrame(makeRx(frame, port.clock));
  runFor(engine, port, 8000000);
  size_t copies = 0;
  for (const std::vector<uint8_t>& sent : port.sent) {
    copies += sent == frame ? 1 : 0;
  }
  TEST_ASSERT_EQUAL_size_t(3, copies);
  TEST_ASSERT_EQUAL_UINT32(1, engine.counters().deliveriesFailed);
  TEST_ASSERT_EQUAL_UINT32(1, findDevice(monitor, kFar)->deliveryFailures);
}

void test_direct_bridge_ack_cancels_the_forward() {
  FakePort port;
  LinkMonitor monitor;
  monitor.setBridge(kBridge);
  monitor.setProxied(kFar, true);
  ProxyEngine engine(port, monitor);
  engine.setProxyMode(true);
  engine.onFrame(makeRx(dataFrame(5, kBridge, kFar), port.clock));
  engine.onFrame(makeRx(enhAck(5, kFar, kBridge), port.clock + 1000));
  runFor(engine, port, 100000);
  TEST_ASSERT_EQUAL_size_t(1, port.sent.size());  // Only our ACK.
}

void test_full_queue_withholds_the_ack() {
  FakePort port;
  LinkMonitor monitor;
  monitor.setBridge(kBridge);
  monitor.setProxied(kFar, true);
  ProxyEngine engine(port, monitor);
  engine.setProxyMode(true);
  for (int seq = 0; seq < 12; seq++) {
    engine.onFrame(makeRx(dataFrame(static_cast<uint8_t>(seq), kBridge, kFar), port.clock));
  }
  TEST_ASSERT_GREATER_THAN_UINT32(0, engine.counters().acksWithheld);
}

void test_leaving_proxy_mode_drops_pending_work() {
  FakePort port;
  LinkMonitor monitor;
  monitor.setBridge(kBridge);
  monitor.setProxied(kFar, true);
  ProxyEngine engine(port, monitor);
  engine.setProxyMode(true);
  engine.onFrame(makeRx(dataFrame(1, kBridge, kFar), port.clock));
  engine.setProxyMode(false);
  runFor(engine, port, 1000000);
  TEST_ASSERT_EQUAL_size_t(0, port.sent.size());
  TEST_ASSERT_EQUAL_size_t(0, engine.uplinksQueued());
}

void test_transmitter_stops_at_the_budget() {
  FakePort port;
  LinkMonitor monitor;
  monitor.setBridge(kBridge);
  monitor.setProxied(kFar, true);
  ProxyEngine engine(port, monitor);
  engine.setProxyMode(true);
  engine.onFrame(makeRx(dataFrame(1, kBridge, kFar), port.clock));
  engine.duty().record(DutyCycleLimiter::kBudgetUs, port.clock);
  runFor(engine, port, 100000);
  TEST_ASSERT_EQUAL_size_t(0, port.sent.size());
  TEST_ASSERT_GREATER_THAN_UINT32(0, engine.counters().dutyBlocked);
}

// Two downlinks in quick succession, with budget for one: the second ACK would be a
// promise the first one already spent.
void test_accepted_work_is_reserved_from_the_budget() {
  FakePort port;
  LinkMonitor monitor;
  monitor.setBridge(kBridge);
  monitor.setProxied(kFar, true);
  ProxyEngine engine(port, monitor);
  engine.setProxyMode(true);
  const std::vector<uint8_t> first = dataFrame(1, kFar, kBridge, 1);
  const std::vector<uint8_t> second = dataFrame(2, kFar, kBridge, 2);
  const uint64_t one =
      airtimeUs(kEnhAckLen) + airtimeUs(first.size()) + (kCslPeriodUs / kWakeStepUs + 1) * airtimeUs(kWakeupLen);
  engine.duty().record(DutyCycleLimiter::kBudgetUs - one - one / 2, port.clock);
  engine.onFrame(makeRx(first, port.clock));
  engine.onFrame(makeRx(second, port.clock + 1000));
  TEST_ASSERT_EQUAL_UINT32(1, engine.counters().acksWithheld);
  TEST_ASSERT_FALSE(engine.idle());
}

void test_proxied_list_is_bounded() {
  LinkMonitor monitor;
  for (size_t i = 0; i < kMaxProxied; i++) {
    const uint8_t prefix[kPrefixLen] = {1, 0, 0, 0, static_cast<uint8_t>(i)};
    TEST_ASSERT_TRUE(monitor.setProxied(prefix, true));
  }
  const uint8_t extra[kPrefixLen] = {2, 0, 0, 0, 0};
  TEST_ASSERT_FALSE(monitor.setProxied(extra, true));
  const uint8_t first[kPrefixLen] = {1, 0, 0, 0, 0};
  TEST_ASSERT_TRUE(monitor.setProxied(first, false));
  TEST_ASSERT_FALSE(monitor.isProxied(first));
  TEST_ASSERT_EQUAL_size_t(kMaxProxied - 1, monitor.proxiedCount());
  TEST_ASSERT_TRUE(monitor.setProxied(extra, true));
}

void test_status_reports_the_proxy() {
  static char buffer[24000];
  FakePort port;
  LinkMonitor monitor;
  monitor.setBridge(kBridge);
  monitor.setProxied(kFar, true);
  ProxyEngine engine(port, monitor);
  engine.setProxyMode(true);
  engine.onFrame(makeRx(dataFrame(1, kBridge, kFar), port.clock));
  runFor(engine, port, 50000);
  StatusInput input = {};
  input.monitor = &monitor;
  input.engine = &engine;
  input.firmware = "1.2.3";
  input.variant = "esp32s3octal";
  input.nowUs = port.clock;
  TEST_ASSERT_TRUE(writeStatusJson(input, buffer, sizeof(buffer)));
  for (const char* key : {"\"mode\":\"proxy\"", "\"proxy_available\":true", "\"proxied\":true", "\"proxied_uplinks\":1",
                          "\"acks_sent\":1", "\"samples\":1", "\"awaiting_ack\":true"}) {
    TEST_ASSERT_NOT_NULL_MESSAGE(std::strstr(buffer, key), key);
  }
}

void test_names_are_set_replaced_and_cleared() {
  NameTable names;
  TEST_ASSERT_TRUE(names.set(kFar, "Bedroom"));
  TEST_ASSERT_EQUAL_STRING("Bedroom", names.find(kFar));
  TEST_ASSERT_TRUE(names.set(kFar, "Study"));
  TEST_ASSERT_EQUAL_STRING("Study", names.find(kFar));
  TEST_ASSERT_TRUE(names.set(kFar, ""));
  TEST_ASSERT_NULL(names.find(kFar));
}

void test_names_that_json_cannot_carry_are_refused() {
  NameTable names;
  TEST_ASSERT_FALSE(names.set(kFar, "a\"b"));
  TEST_ASSERT_FALSE(names.set(kFar, "a\\b"));
  TEST_ASSERT_FALSE(names.set(kFar, "a\nb"));
  TEST_ASSERT_FALSE(names.set(kFar, "1234567890123456789012345"));
  TEST_ASSERT_TRUE(names.set(kFar, "123456789012345678901234"));
  TEST_ASSERT_TRUE(names.set(kBridge, "Slaapkamer \xC3\xA9\xC3\xA9n"));
}

void test_full_name_table_refuses_a_new_device() {
  NameTable names;
  for (size_t i = 0; i < NameTable::kCapacity; i++) {
    const uint8_t prefix[kPrefixLen] = {1, 0, 0, 0, static_cast<uint8_t>(i)};
    TEST_ASSERT_TRUE(names.set(prefix, "x"));
  }
  TEST_ASSERT_FALSE(names.set(kFar, "x"));
  const uint8_t known[kPrefixLen] = {1, 0, 0, 0, 3};
  TEST_ASSERT_TRUE(names.set(known, "renamed"));
}

void test_status_shows_the_name_and_the_last_downlink() {
  static char buffer[24000];
  LinkMonitor monitor;
  monitor.setBridge(kBridge);
  NameTable names;
  names.set(kFar, "Bedroom");
  monitor.onFrame(makeRx(dataFrame(2, kFar, kBridge), 1000000));
  monitor.onFrame(makeRx(enhAck(2, kBridge, kFar), 1006000));
  StatusInput input = {};
  input.monitor = &monitor;
  input.names = &names;
  input.firmware = "1.2.3";
  input.variant = "esp32s3octal";
  input.nowUs = 3500000;
  TEST_ASSERT_TRUE(writeStatusJson(input, buffer, sizeof(buffer)));
  TEST_ASSERT_NOT_NULL(std::strstr(buffer, "\"name\":\"Bedroom\",\"downlink_age_s\":2.5"));
  TEST_ASSERT_NOT_NULL(std::strstr(buffer, "\"name\":null,\"downlink_age_s\":null"));
}

void test_worst_case_status_with_names_fits() {
  static char buffer[kStatusJsonCapacity];
  LinkMonitor monitor;
  NameTable names;
  monitor.setBridge(kBridge);
  for (int i = 0; i < 60; i++) {
    const uint8_t prefix[kPrefixLen] = {0xA1, 0, 0, 0, static_cast<uint8_t>(i)};
    monitor.onFrame(makeRx(dataFrame(static_cast<uint8_t>(i), prefix, kBridge), 1000000));
    names.set(prefix, "123456789012345678901234");
  }
  StatusInput input = {};
  input.monitor = &monitor;
  input.names = &names;
  input.firmware = "1.2.3";
  input.variant = "esp32s3octal";
  input.nowUs = 2000000;
  TEST_ASSERT_TRUE(writeStatusJson(input, buffer, sizeof(buffer)));
}

// Web guard -------------------------------------------------------------------

void test_host_names_of_the_node_are_allowed() {
  const char* allowed[] = {"192.168.1.20",           "192.168.1.20:80",      "tado-gado-ab12",
                           "tado-gado-ab12.local",   "TADO-GADO-AB12.Local", "tado-gado-ab12.lan",
                           "tado-gado-ab12.local:80"};
  for (const char* host : allowed) {
    TEST_ASSERT_TRUE_MESSAGE(hostAllowed(host, "192.168.1.20", "tado-gado-ab12"), host);
  }
}

void test_other_host_names_are_refused() {
  const char* refused[] = {"",
                           ":80",
                           "evil.example",
                           "192.168.1.2",
                           "192.168.1.200",
                           "tado-gado-ab1",
                           "tado-gado-ab123",
                           "tado-gado",
                           "evil.tado-gado-ab12.local"};
  for (const char* host : refused) {
    TEST_ASSERT_FALSE_MESSAGE(hostAllowed(host, "192.168.1.20", "tado-gado-ab12"), host);
  }
  TEST_ASSERT_FALSE(hostAllowed(".evil.example", "192.168.1.20", ""));  // Before the name is set.
}

void test_wrong_tokens_wait_longer_each_time() {
  TokenLockout lockout;
  TEST_ASSERT_FALSE(lockout.locked(1, 0));
  lockout.failed(1, 1000);
  TEST_ASSERT_TRUE(lockout.locked(1, 1499));
  TEST_ASSERT_FALSE(lockout.locked(1, 1500));
  lockout.failed(1, 1500);
  TEST_ASSERT_TRUE(lockout.locked(1, 2499));
  TEST_ASSERT_FALSE(lockout.locked(1, 2500));
  for (int i = 0; i < 40; i++) {
    lockout.failed(1, 10000);
  }
  TEST_ASSERT_TRUE(lockout.locked(1, 10000 + TokenLockout::kMaxDelayMs - 1));
  TEST_ASSERT_FALSE(lockout.locked(1, 10000 + TokenLockout::kMaxDelayMs));
  lockout.succeeded(1);
  lockout.failed(1, 50000);
  TEST_ASSERT_FALSE(lockout.locked(1, 50500));  // Back to the first delay.
}

void test_one_client_cannot_lock_out_another() {
  TokenLockout lockout;
  for (uint32_t client = 1; client <= 3 * TokenLockout::kClients; client++) {
    lockout.failed(client, 1000);
  }
  TEST_ASSERT_FALSE(lockout.locked(1000, 1000));
  TEST_ASSERT_TRUE(lockout.locked(3 * TokenLockout::kClients, 1000));
}

// Pair ------------------------------------------------------------------------

std::vector<uint8_t> hexBytes(const char* hex) { return fromHex(hex); }

void test_sha256_known_answers() {
  uint8_t out[kSha256Len];
  const char* abc = "abc";
  sha256(reinterpret_cast<const uint8_t*>(abc), 3, out);
  std::vector<uint8_t> expected = hexBytes("ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
  TEST_ASSERT_EQUAL_MEMORY(expected.data(), out, kSha256Len);

  sha256(nullptr, 0, out);
  expected = hexBytes("e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
  TEST_ASSERT_EQUAL_MEMORY(expected.data(), out, kSha256Len);

  const char* twoBlocks = "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq";
  sha256(reinterpret_cast<const uint8_t*>(twoBlocks), 56, out);
  expected = hexBytes("248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1");
  TEST_ASSERT_EQUAL_MEMORY(expected.data(), out, kSha256Len);
}

void test_hmac_known_answer() {  // RFC 4231, test case 2.
  const char* key = "Jefe";
  const char* data = "what do ya want for nothing?";
  uint8_t out[kSha256Len];
  hmacSha256(reinterpret_cast<const uint8_t*>(key), 4, reinterpret_cast<const uint8_t*>(data), 28, out);
  const std::vector<uint8_t> expected = hexBytes("5bdcc146bf60754e6a042426089575c75a003f089d2739839dec58b964ec3843");
  TEST_ASSERT_EQUAL_MEMORY(expected.data(), out, kSha256Len);
}

void test_pbkdf2_known_answers() {  // PBKDF2-HMAC-SHA256, P = "password", S = "salt".
  const uint8_t* password = reinterpret_cast<const uint8_t*>("password");
  const uint8_t* salt = reinterpret_cast<const uint8_t*>("salt");
  uint8_t out[kSha256Len];
  pbkdf2Sha256(password, 8, salt, 4, 1, out);
  std::vector<uint8_t> expected = hexBytes("120fb6cffcf8b32c43e7225256c4f837a86548c92ccc35480805987cb70be17b");
  TEST_ASSERT_EQUAL_MEMORY(expected.data(), out, kSha256Len);
  pbkdf2Sha256(password, 8, salt, 4, 2, out);
  expected = hexBytes("ae4d0c95af6b46d32d0adff928f06dd02a303f8ef3c251dfd6e2d85a95474c43");
  TEST_ASSERT_EQUAL_MEMORY(expected.data(), out, kSha256Len);
  pbkdf2Sha256(password, 8, salt, 4, 4096, out);
  expected = hexBytes("c5e478d59288c841aa530db6845c4c8d962893a001ce4e11a4963873aa98134a");
  TEST_ASSERT_EQUAL_MEMORY(expected.data(), out, kSha256Len);
}

void test_pair_key_is_derived_from_the_token() {
  uint8_t first[kPairKeyLen];
  uint8_t again[kPairKeyLen];
  uint8_t other[kPairKeyLen];
  derivePairKey("correct horse battery", first, 10);
  derivePairKey("correct horse battery", again, 10);
  derivePairKey("correct horse batterz", other, 10);
  TEST_ASSERT_EQUAL_MEMORY(first, again, kPairKeyLen);
  TEST_ASSERT_FALSE(equalConstantTime(first, other, kPairKeyLen));
}

void test_constant_time_compare() {
  const uint8_t a[] = {1, 2, 3};
  const uint8_t b[] = {1, 2, 4};
  TEST_ASSERT_TRUE(equalConstantTime(a, a, 3));
  TEST_ASSERT_FALSE(equalConstantTime(a, b, 3));
  TEST_ASSERT_TRUE(equalConstantTime(a, b, 2));
}

const uint8_t kSecret[] = {'t', 'o', 'k', 'e', 'n'};

PairHeader sampleHeader() {
  PairHeader header = {};
  header.type = PairType::Uplink;
  header.nodeId[0] = 7;
  header.senderBoot = 11;
  header.receiverBoot = 22;
  header.counter = 33;
  return header;
}

void test_pair_datagram_round_trip() {
  const uint8_t payload[] = {1, 2, 3};
  uint8_t datagram[kPairMaxDatagram];
  const size_t length = sealPair(kSecret, sizeof(kSecret), sampleHeader(), payload, sizeof(payload), datagram);
  TEST_ASSERT_EQUAL_size_t(kPairHeaderLen + 3 + kPairMacLen, length);
  PairHeader header;
  const uint8_t* opened = nullptr;
  size_t openedLength = 0;
  TEST_ASSERT_TRUE(openPair(kSecret, sizeof(kSecret), datagram, length, header, opened, openedLength));
  TEST_ASSERT_TRUE(header.type == PairType::Uplink);
  TEST_ASSERT_EQUAL_UINT32(11, header.senderBoot);
  TEST_ASSERT_EQUAL_UINT32(22, header.receiverBoot);
  TEST_ASSERT_EQUAL_UINT32(33, header.counter);
  TEST_ASSERT_EQUAL_size_t(3, openedLength);
  TEST_ASSERT_EQUAL_MEMORY(payload, opened, 3);
}

void test_forged_datagrams_are_refused() {
  const uint8_t payload[] = {1, 2, 3};
  uint8_t datagram[kPairMaxDatagram];
  const size_t length = sealPair(kSecret, sizeof(kSecret), sampleHeader(), payload, sizeof(payload), datagram);
  PairHeader header;
  const uint8_t* opened = nullptr;
  size_t openedLength = 0;
  const uint8_t otherSecret[] = {'o', 't', 'h', 'e', 'r'};
  TEST_ASSERT_FALSE(openPair(otherSecret, sizeof(otherSecret), datagram, length, header, opened, openedLength));
  for (size_t i = 0; i < length; i++) {
    datagram[i] ^= 0x01;
    TEST_ASSERT_FALSE(openPair(kSecret, sizeof(kSecret), datagram, length, header, opened, openedLength));
    datagram[i] ^= 0x01;
  }
  TEST_ASSERT_FALSE(openPair(kSecret, sizeof(kSecret), datagram, 5, header, opened, openedLength));
  TEST_ASSERT_FALSE(openPair(kSecret, sizeof(kSecret), datagram, length - 1, header, opened, openedLength));
}

class FakeTransport : public PairTransport {
 public:
  void broadcast(const uint8_t* data, size_t length) override { out.emplace_back(data, data + length); }
  void sendToPeer(const uint8_t* data, size_t length) override { out.emplace_back(data, data + length); }
  void rememberPeer() override {}

  std::vector<std::vector<uint8_t>> out;
};

struct Node {
  explicit Node(uint8_t id, uint32_t boot) : engine(port, monitor), pair(transport, engine, monitor) {
    nodeId[0] = id;
    engine.setPeer(&pair);
    pair.begin(kSecret, sizeof(kSecret), nodeId, boot);
  }

  void hearBridge(float rssi) {
    RxFrame rx = makeRx(dataFrame(1, kFar, kBridge, static_cast<uint8_t>(++tag)), port.clock);
    rx.rssiDbm = rssi;
    monitor.setBridge(kBridge);
    engine.onFrame(rx);
  }

  FakePort port;
  LinkMonitor monitor;
  ProxyEngine engine;
  FakeTransport transport;
  PairManager pair;
  uint8_t nodeId[kNodeIdLen] = {};
  int tag = 0;
};

// Moves the datagrams each node sent to the other one.
void deliver(Node& from, Node& to) {
  for (const std::vector<uint8_t>& datagram : from.transport.out) {
    to.pair.onDatagram(datagram.data(), datagram.size(), to.port.clock);
  }
  from.transport.out.clear();
}

void advance(Node& a, Node& b, uint64_t durationUs, bool linked = true) {
  const uint64_t end = a.port.clock + durationUs;
  while (a.port.clock < end) {
    a.port.clock += 100000;
    b.port.clock += 100000;
    a.pair.tick(a.port.clock);
    b.pair.tick(b.port.clock);
    if (linked) {
      deliver(a, b);
      deliver(b, a);
    } else {
      a.transport.out.clear();
      b.transport.out.clear();
    }
  }
}

void test_strongest_node_becomes_the_bridge_node() {
  Node a(1, 100);
  Node b(2, 200);
  a.hearBridge(-90.0f);
  b.hearBridge(-60.0f);
  advance(a, b, 5000000);
  TEST_ASSERT_TRUE(a.pair.role() == Role::Valve);
  TEST_ASSERT_TRUE(b.pair.role() == Role::Bridge);
}

void test_a_node_that_hears_no_bridge_becomes_the_valve_node() {
  Node a(1, 100);
  Node b(2, 200);
  b.hearBridge(-95.0f);
  advance(a, b, 5000000);
  TEST_ASSERT_TRUE(a.pair.role() == Role::Valve);
  TEST_ASSERT_TRUE(b.pair.role() == Role::Bridge);
  TEST_ASSERT_TRUE(a.monitor.hasBridge());  // It learned the bridge from its partner.
  TEST_ASSERT_EQUAL_MEMORY(kBridge, a.monitor.bridge(), kPrefixLen);
}

void test_no_roles_while_neither_hears_the_bridge() {
  Node a(1, 100);
  Node b(2, 200);
  advance(a, b, 5000000);
  TEST_ASSERT_FALSE(a.pair.rolesDecided());
  TEST_ASSERT_FALSE(a.pair.requestProxy(true, a.port.clock));
  TEST_ASSERT_FALSE(a.pair.wanted());
}

void test_roles_do_not_flip_on_a_few_db() {
  Node a(1, 100);
  Node b(2, 200);
  a.hearBridge(-70.0f);
  b.hearBridge(-71.0f);
  advance(a, b, 5000000);
  TEST_ASSERT_TRUE(a.pair.role() == Role::Bridge);
  for (int i = 0; i < 20; i++) {
    b.hearBridge(-69.0f);
  }
  advance(a, b, 5000000);
  TEST_ASSERT_TRUE(a.pair.role() == Role::Bridge);
  for (int i = 0; i < 20; i++) {
    b.hearBridge(-50.0f);
  }
  advance(a, b, 5000000);
  TEST_ASSERT_TRUE(a.pair.role() == Role::Valve);
  TEST_ASSERT_TRUE(b.pair.role() == Role::Bridge);
}

void pairUp(Node& a, Node& b) {
  a.hearBridge(-60.0f);
  advance(a, b, 5000000);
}

void test_nodes_stay_in_monitor_until_asked() {
  Node a(1, 100);
  Node b(2, 200);
  pairUp(a, b);
  advance(a, b, 30000000);
  TEST_ASSERT_FALSE(a.pair.wanted());
  TEST_ASSERT_FALSE(a.engine.proxyMode());
  TEST_ASSERT_FALSE(b.engine.proxyMode());
}

void test_one_request_starts_both_nodes_with_their_roles() {
  Node a(1, 100);
  Node b(2, 200);
  pairUp(a, b);
  TEST_ASSERT_TRUE(b.pair.requestProxy(true, b.port.clock));
  advance(a, b, 1000000);
  TEST_ASSERT_TRUE(a.engine.proxyMode());
  TEST_ASSERT_TRUE(b.engine.proxyMode());
  TEST_ASSERT_TRUE(a.engine.role() == Role::Bridge);
  TEST_ASSERT_TRUE(b.engine.role() == Role::Valve);
}

void test_one_request_stops_both_nodes() {
  Node a(1, 100);
  Node b(2, 200);
  pairUp(a, b);
  a.pair.requestProxy(true, a.port.clock);
  advance(a, b, 1000000);
  b.pair.requestProxy(false, b.port.clock);
  advance(a, b, 1000000);
  TEST_ASSERT_FALSE(a.engine.proxyMode());
  TEST_ASSERT_FALSE(b.engine.proxyMode());
  TEST_ASSERT_FALSE(a.pair.wanted());
}

void test_no_transmission_before_the_partner_listens() {
  Node a(1, 100);
  Node b(2, 200);
  pairUp(a, b);
  a.pair.requestProxy(true, a.port.clock);
  a.pair.tick(a.port.clock);
  TEST_ASSERT_FALSE(a.engine.proxyMode());  // Not before the partner answered.
}

void test_start_gives_up_if_the_partner_never_answers() {
  Node a(1, 100);
  Node b(2, 200);
  pairUp(a, b);
  a.pair.requestProxy(true, a.port.clock);
  a.transport.out.clear();
  for (int i = 0; i < 120; i++) {
    a.port.clock += 100000;
    b.port.clock += 100000;
    a.pair.tick(a.port.clock);
    b.pair.tick(b.port.clock);
    a.transport.out.clear();  // Nothing of a arrives, so b does not join.
    deliver(b, a);
  }
  TEST_ASSERT_FALSE(a.pair.wanted());
  TEST_ASSERT_FALSE(a.engine.proxyMode());
}

void test_lost_link_pauses_and_a_returning_link_resumes() {
  Node a(1, 100);
  Node b(2, 200);
  pairUp(a, b);
  a.pair.requestProxy(true, a.port.clock);
  advance(a, b, 1000000);
  advance(a, b, 12000000, false);
  TEST_ASSERT_FALSE(a.engine.proxyMode());
  TEST_ASSERT_FALSE(b.engine.proxyMode());
  TEST_ASSERT_TRUE(a.pair.wanted());
  advance(a, b, 3000000);
  TEST_ASSERT_TRUE(a.engine.proxyMode());
  TEST_ASSERT_TRUE(b.engine.proxyMode());
}

void test_a_restart_of_one_node_stops_the_other() {
  Node a(1, 100);
  Node b(2, 200);
  pairUp(a, b);
  a.pair.requestProxy(true, a.port.clock);
  advance(a, b, 1000000);
  Node restarted(2, 201);
  restarted.port.clock = a.port.clock;
  advance(a, restarted, 3000000);
  TEST_ASSERT_FALSE(a.pair.wanted());
  TEST_ASSERT_FALSE(a.engine.proxyMode());
  TEST_ASSERT_FALSE(restarted.engine.proxyMode());
  TEST_ASSERT_EQUAL_UINT32(1, a.pair.peerRestarts());
}

void test_replayed_frame_is_refused() {
  Node a(1, 100);
  Node b(2, 200);
  pairUp(a, b);
  a.pair.requestProxy(true, a.port.clock);
  advance(a, b, 1000000);
  b.pair.sendFrame(true, dataFrame(9, kBridge, kFar).data(), dataFrame(9, kBridge, kFar).size());
  const std::vector<uint8_t> datagram = b.transport.out.back();
  deliver(b, a);
  const size_t queued = a.engine.uplinksQueued();
  TEST_ASSERT_EQUAL_size_t(1, queued);
  const uint32_t rejected = a.pair.rejected();
  a.pair.onDatagram(datagram.data(), datagram.size(), a.port.clock);
  TEST_ASSERT_EQUAL_UINT32(rejected + 1, a.pair.rejected());
  TEST_ASSERT_EQUAL_size_t(queued, a.engine.uplinksQueued());
}

void test_frame_for_an_earlier_boot_is_refused() {
  Node a(1, 100);
  Node b(2, 200);
  pairUp(a, b);
  PairHeader header = {};
  header.type = PairType::Mode;
  header.nodeId[0] = 2;
  header.senderBoot = 200;
  header.receiverBoot = 99;  // Not the boot of a.
  header.counter = 1000000;
  const uint8_t on = 1;
  uint8_t datagram[kPairMaxDatagram];
  const size_t length = sealPair(kSecret, sizeof(kSecret), header, &on, 1, datagram);
  a.pair.onDatagram(datagram, length, a.port.clock);
  TEST_ASSERT_FALSE(a.pair.wanted());
  TEST_ASSERT_EQUAL_UINT32(1, a.pair.rejected());
}

// A LAN host replays Hellos and a Mode(1) it recorded: an old boot first, to reset
// the counters, then the current boot, then the Mode. None of it may count.
void test_replayed_hellos_of_an_older_boot_cannot_restart_a_pair() {
  Node a(1, 100);
  Node old(2, 199);
  old.hearBridge(-60.0f);
  old.pair.tick(old.port.clock);
  const std::vector<uint8_t> oldHello = old.transport.out.front();

  Node b(2, 200);
  b.port.clock = a.port.clock;
  pairUp(a, b);
  b.transport.out.clear();
  b.port.clock += 2100000;
  b.pair.tick(b.port.clock);
  const std::vector<uint8_t> currentHello = b.transport.out.front();
  b.transport.out.clear();
  b.pair.requestProxy(true, b.port.clock);
  const std::vector<uint8_t> modeOn = b.transport.out.back();
  advance(a, b, 1000000);
  b.pair.requestProxy(false, b.port.clock);
  advance(a, b, 1000000);
  TEST_ASSERT_FALSE(a.pair.wanted());

  const uint32_t restarts = a.pair.peerRestarts();
  const uint32_t rejected = a.pair.rejected();
  a.pair.onDatagram(oldHello.data(), oldHello.size(), a.port.clock);
  a.pair.onDatagram(currentHello.data(), currentHello.size(), a.port.clock);
  a.pair.onDatagram(modeOn.data(), modeOn.size(), a.port.clock);
  a.pair.tick(a.port.clock);
  TEST_ASSERT_FALSE(a.pair.wanted());
  TEST_ASSERT_FALSE(a.engine.proxyMode());
  TEST_ASSERT_EQUAL_UINT32(restarts, a.pair.peerRestarts());
  TEST_ASSERT_EQUAL_UINT32(rejected + 3, a.pair.rejected());
}

void test_a_third_node_is_ignored() {
  Node a(1, 100);
  Node b(2, 200);
  Node c(3, 300);
  pairUp(a, b);
  c.pair.tick(1);
  deliver(c, a);
  TEST_ASSERT_EQUAL_MEMORY(b.nodeId, a.pair.peerId(), kNodeIdLen);
}

void test_proxied_list_travels_to_the_partner() {
  Node a(1, 100);
  Node b(2, 200);
  pairUp(a, b);
  a.monitor.setProxied(kFar, true);
  a.pair.syncProxied();
  advance(a, b, 1000000);
  TEST_ASSERT_TRUE(b.monitor.isProxied(kFar));
  TEST_ASSERT_TRUE(b.pair.takeProxiedChanged());
  TEST_ASSERT_FALSE(b.pair.takeProxiedChanged());
  a.monitor.setProxied(kFar, false);
  a.pair.syncProxied();
  advance(a, b, 1000000);
  TEST_ASSERT_FALSE(b.monitor.isProxied(kFar));
}

void test_a_node_without_a_partner_proxies_alone() {
  Node a(1, 100);
  TEST_ASSERT_TRUE(a.pair.requestProxy(true, a.port.clock));
  a.pair.tick(a.port.clock);
  TEST_ASSERT_TRUE(a.engine.proxyMode());
  TEST_ASSERT_TRUE(a.engine.role() == Role::Single);
}

void test_pairing_off_without_a_token() {
  FakePort port;
  LinkMonitor monitor;
  ProxyEngine engine(port, monitor);
  FakeTransport transport;
  PairManager pair(transport, engine, monitor);
  const uint8_t id[kNodeIdLen] = {1};
  pair.begin(nullptr, 0, id, 1);
  pair.tick(5000000);
  TEST_ASSERT_FALSE(pair.enabled());
  TEST_ASSERT_EQUAL_size_t(0, transport.out.size());
  TEST_ASSERT_TRUE(pair.requestProxy(true, 5000000));
}

void test_frame_goes_from_valve_node_to_bridge_node_and_on_air() {
  Node a(1, 100);
  Node b(2, 200);
  pairUp(a, b);
  a.monitor.setProxied(kFar, true);
  a.pair.syncProxied();
  a.pair.requestProxy(true, a.port.clock);
  advance(a, b, 1000000);
  const std::vector<uint8_t> frame = dataFrame(42, kBridge, kFar, 7);
  b.engine.onFrame(makeRx(frame, b.port.clock));
  TEST_ASSERT_EQUAL_size_t(0, b.engine.uplinksQueued());
  deliver(b, a);
  TEST_ASSERT_EQUAL_size_t(1, a.engine.uplinksQueued());
  runFor(a.engine, a.port, 10000);
  TEST_ASSERT_TRUE(a.port.sent.back() == frame);
  runFor(b.engine, b.port, 10000);
  EnhAck ack;
  TEST_ASSERT_TRUE(parseEnhAck(b.port.sent.back().data(), b.port.sent.back().size(), ack));
  TEST_ASSERT_EQUAL_UINT8(42, ack.seq);
}

void test_status_shows_the_pair() {
  static char buffer[24000];
  Node a(1, 100);
  Node b(2, 200);
  pairUp(a, b);
  a.pair.requestProxy(true, a.port.clock);
  StatusInput input = {};
  input.monitor = &a.monitor;
  input.engine = &a.engine;
  PairStatus pair = a.pair.status(a.port.clock);
  input.pair = &pair;
  input.firmware = "1.2.3";
  input.variant = "esp32s3octal";
  input.nowUs = a.port.clock;
  TEST_ASSERT_TRUE(writeStatusJson(input, buffer, sizeof(buffer)));
  for (const char* key : {"\"mode\":\"proxy\"", "\"node_mode\":\"waiting\"", "\"role\":\"bridge\"",
                          "\"peer\":\"020000000000\"", "\"peer_alive\":true", "\"peer_bridge_rssi\":null"}) {
    TEST_ASSERT_NOT_NULL_MESSAGE(std::strstr(buffer, key), key);
  }
  advance(a, b, 1000000);
  input.nowUs = a.port.clock;
  pair = a.pair.status(a.port.clock);
  TEST_ASSERT_TRUE(writeStatusJson(input, buffer, sizeof(buffer)));
  TEST_ASSERT_NOT_NULL(std::strstr(buffer, "\"node_mode\":\"bridge\""));
}

void test_json_fields_for_the_web_api() {
  char text[16];
  bool flag = false;
  uint8_t prefix[kPrefixLen];
  const char* body = "{\"device\": \"a000000002\", \"enabled\": true, \"mode\":\"proxy\"}";
  TEST_ASSERT_TRUE(jsonStringField(body, "mode", text, sizeof(text)));
  TEST_ASSERT_EQUAL_STRING("proxy", text);
  TEST_ASSERT_TRUE(jsonBoolField(body, "enabled", flag));
  TEST_ASSERT_TRUE(flag);
  TEST_ASSERT_TRUE(jsonStringField(body, "device", text, sizeof(text)));
  TEST_ASSERT_TRUE(parsePrefix(text, prefix));
  TEST_ASSERT_EQUAL_MEMORY(kFar, prefix, kPrefixLen);
  TEST_ASSERT_FALSE(parsePrefix("a00000000", prefix));
  TEST_ASSERT_FALSE(parsePrefix("zz00000002", prefix));
  TEST_ASSERT_FALSE(jsonBoolField("{\"enabled\": 1}", "enabled", flag));
  TEST_ASSERT_FALSE(jsonStringField("{\"mode\": 3}", "mode", text, sizeof(text)));
  char tiny[3];
  TEST_ASSERT_FALSE(jsonStringField(body, "mode", tiny, sizeof(tiny)));
}

void test_reset_button_acts_when_a_long_press_is_released() {
  ResetButton button;
  TEST_ASSERT_FALSE(button.update(false, 0));
  TEST_ASSERT_FALSE(button.update(true, 1000));
  TEST_ASSERT_FALSE(button.update(true, 4000));
  TEST_ASSERT_TRUE(button.update(false, 1000 + ResetButton::kMinHoldMs));
  TEST_ASSERT_FALSE(button.update(false, 4020));  // Only once.

  ResetButton pressedFromTheStart;
  TEST_ASSERT_FALSE(pressedFromTheStart.update(true, 0));
  TEST_ASSERT_TRUE(pressedFromTheStart.update(false, ResetButton::kMaxHoldMs));
}

void test_reset_button_ignores_a_press_of_the_wrong_length() {
  ResetButton button;
  button.update(true, 1000);
  TEST_ASSERT_FALSE(button.update(false, 999 + ResetButton::kMinHoldMs));
  // Two short presses are not one long press.
  button.update(true, 4500);
  TEST_ASSERT_FALSE(button.update(false, 5500));

  ResetButton heldByTheUsbPort;
  heldByTheUsbPort.update(true, 1000);
  TEST_ASSERT_FALSE(heldByTheUsbPort.update(false, 1001 + ResetButton::kMaxHoldMs));
}

void test_reset_button_ignores_a_press_after_the_window() {
  ResetButton button;
  button.update(true, ResetButton::kWindowMs);
  TEST_ASSERT_TRUE(button.done(ResetButton::kWindowMs));
  TEST_ASSERT_FALSE(button.update(false, ResetButton::kWindowMs + 5000));
}

void test_reset_button_waits_for_a_press_that_began_in_the_window() {
  ResetButton button;
  TEST_ASSERT_FALSE(button.done(ResetButton::kWindowMs - 1));
  button.update(true, ResetButton::kWindowMs - 1);
  TEST_ASSERT_FALSE(button.done(ResetButton::kWindowMs + 5000));
  TEST_ASSERT_TRUE(button.update(false, ResetButton::kWindowMs + 5000));
  TEST_ASSERT_TRUE(button.done(ResetButton::kWindowMs + 5000));
}

// Windows, trains and the shared view of a pair -----------------------------------

const uint64_t kMinute = 60000000;

void hearUplink(LinkMonitor& monitor, uint8_t seq, uint64_t atUs, bool acked) {
  monitor.onFrame(makeRx(dataFrame(seq, kBridge, kFar, seq), atUs));
  if (acked) {
    monitor.onFrame(makeRx(enhAck(seq, kFar, kBridge), atUs + 6000));
  }
  monitor.tick(atUs + 30000);
}

// A train of `spanUs` for kFar, the bridge's frame behind it, and maybe kFar's answer.
void hearDownlink(LinkMonitor& monitor, uint8_t seq, uint64_t atUs, uint64_t spanUs, bool acked) {
  for (uint64_t t = 0; t <= spanUs; t += kWakeStepUs) {
    monitor.onFrame(makeRx(wakeup(seq), atUs + t));
  }
  const uint64_t dataAt = atUs + spanUs + 3000;
  monitor.onFrame(makeRx(dataFrame(seq, kFar, kBridge, seq), dataAt));
  if (acked) {
    monitor.onFrame(makeRx(enhAck(seq, kBridge, kFar), dataAt + 6000));
  }
  monitor.tick(dataAt + 30000);
}

void test_windows_keep_the_last_hour_and_the_last_day() {
  LinkMonitor monitor;
  monitor.setBridge(kBridge);
  hearUplink(monitor, 1, 1 * kMinute, true);
  hearUplink(monitor, 2, 2 * kMinute, false);
  const DeviceLink* far = monitor.findLink(kFar);
  const LinkTotals hour = monitor.hourTotals(*far);
  TEST_ASSERT_EQUAL_UINT32(2, hour.uplinks);
  TEST_ASSERT_EQUAL_UINT32(1, hour.uplinksAcked);
  TEST_ASSERT_EQUAL_UINT32(1, hour.uplinksUnacked);
  TEST_ASSERT_EQUAL_UINT64(2 * airtimeUs(dataFrame(1, kBridge, kFar).size()), hour.airtimeUs);
  monitor.tick(65 * kMinute);
  TEST_ASSERT_EQUAL_UINT32(2, monitor.hourTotals(*far).uplinks);  // Dropped a bucket late.
  monitor.tick(70 * kMinute);
  TEST_ASSERT_EQUAL_UINT32(0, monitor.hourTotals(*far).uplinks);
  TEST_ASSERT_EQUAL_UINT32(2, monitor.dayTotals(*far).uplinks);
  monitor.tick(26 * 60 * kMinute);
  TEST_ASSERT_EQUAL_UINT32(0, monitor.dayTotals(*far).uplinks);
  TEST_ASSERT_EQUAL_UINT32(2, far->uplinks);  // Since boot.
}

void test_windows_start_again_when_the_proxy_starts_or_stops() {
  FakePort port;
  LinkMonitor monitor;
  ProxyEngine engine(port, monitor);
  monitor.setBridge(kBridge);
  hearUplink(monitor, 1, port.clock, true);
  const DeviceLink* far = monitor.findLink(kFar);
  TEST_ASSERT_EQUAL_UINT32(1, monitor.hourTotals(*far).uplinks);
  engine.setProxyMode(true);
  TEST_ASSERT_EQUAL_UINT32(0, monitor.hourTotals(*far).uplinks);
  TEST_ASSERT_EQUAL_UINT32(0, monitor.dayTotals(*far).uplinks);
  TEST_ASSERT_EQUAL_UINT64(port.clock, monitor.windowsSinceUs());
  TEST_ASSERT_EQUAL_UINT32(1, far->uplinks);
  port.clock += kMinute;
  engine.setProxyMode(true);  // No change of mode, so no reset.
  TEST_ASSERT_EQUAL_UINT64(port.clock - kMinute, monitor.windowsSinceUs());
  engine.setProxyMode(false);
  TEST_ASSERT_EQUAL_UINT64(port.clock, monitor.windowsSinceUs());
}

void test_downlinks_behind_long_and_short_trains_are_told_apart() {
  LinkMonitor monitor;
  monitor.setBridge(kBridge);
  hearDownlink(monitor, 1, 1000000, 800000, true);
  hearDownlink(monitor, 2, 3000000, 800000, false);
  hearDownlink(monitor, 3, 5000000, 40000, false);
  hearDownlink(monitor, 4, 7000000, 40000, false);
  const DeviceLink* far = monitor.findLink(kFar);
  TEST_ASSERT_EQUAL_UINT32(1, far->downlinksAcked);
  TEST_ASSERT_EQUAL_UINT32(3, far->downlinksUnacked);
  TEST_ASSERT_EQUAL_UINT32(1, far->longTrainAcked);
  TEST_ASSERT_EQUAL_UINT32(1, far->longTrainUnacked);
  const LinkTotals hour = monitor.hourTotals(*far);
  TEST_ASSERT_EQUAL_UINT32(1, hour.longTrainAcked);
  TEST_ASSERT_EQUAL_UINT32(1, hour.longTrainUnacked);
  TEST_ASSERT_EQUAL_UINT32(4, hour.downlinks);
}

void test_wakeup_trains_are_counted_by_sender() {
  LinkMonitor monitor;
  monitor.setBridge(kBridge);
  const uint64_t span = 400000;
  const uint64_t train = (span / kWakeStepUs + 1) * airtimeUs(kWakeupLen);
  hearDownlink(monitor, 1, 1000000, span, false);
  // No frame follows this one.
  for (uint64_t t = 0; t <= span; t += kWakeStepUs) {
    monitor.onFrame(makeRx(wakeup(2), 3000000 + t));
  }
  monitor.tick(3600000);
  // The other node's train: its frame is one this node handed over.
  const std::vector<uint8_t> handed = dataFrame(3, kFar, kBridge, 99);
  monitor.noteHandedOver(handed.data(), handed.size(), 5000000);
  for (uint64_t t = 0; t <= span; t += kWakeStepUs) {
    monitor.onFrame(makeRx(wakeup(3), 5000000 + t));
  }
  monitor.onFrame(makeRx(handed, 5000000 + span + 3000));
  monitor.tick(6000000);

  const TrainTotals trains = monitor.hourTrains();
  TEST_ASSERT_EQUAL_UINT64(train, trains.bridgeUs);
  TEST_ASSERT_EQUAL_UINT64(train, trains.unknownUs);
  TEST_ASSERT_EQUAL_UINT64(train, trains.peerUs);
  TEST_ASSERT_EQUAL_UINT64(3 * train, monitor.hourTotals(*monitor.findLink(kFar)).trainUs);
  const LinkTotals bridge = monitor.hourTotals(*monitor.findLink(kBridge));
  TEST_ASSERT_EQUAL_UINT64(train + airtimeUs(handed.size()), bridge.airtimeUs);
  TEST_ASSERT_EQUAL_UINT64(train, monitor.dayTrains().bridgeUs);
}

void test_a_train_for_another_device_is_not_claimed() {
  LinkMonitor monitor;
  monitor.setBridge(kBridge);
  for (uint64_t t = 0; t <= 40000; t += kWakeStepUs) {
    monitor.onFrame(makeRx(wakeup(5), 1000000 + t));
  }
  const uint8_t other[kPrefixLen] = {0xC0, 0x01, 0, 0, 3};
  monitor.onFrame(makeRx(dataFrame(5, other, kBridge, 5), 1043000));
  monitor.tick(1200000);
  TEST_ASSERT_EQUAL_UINT64(0, monitor.hourTrains().bridgeUs);
  TEST_ASSERT_GREATER_THAN_UINT64(0, monitor.hourTrains().unknownUs);
}

void test_an_answer_heard_only_by_the_other_node_counts() {
  LinkMonitor monitor;
  monitor.setBridge(kBridge);
  monitor.setPeerLinked(true);
  monitor.onFrame(makeRx(dataFrame(1, kFar, kBridge, 1), 1000000));
  monitor.tick(1100000);
  const DeviceLink* far = monitor.findLink(kFar);
  TEST_ASSERT_EQUAL_UINT32(0, far->downlinksUnacked);  // It waits for the other node.
  monitor.notePeerAck(kFar, 1, false, 1500000);
  TEST_ASSERT_EQUAL_UINT32(1, far->downlinksAcked);
  TEST_ASSERT_EQUAL_UINT32(1, far->ackedByPeer);
  TEST_ASSERT_EQUAL_UINT32(1, monitor.hourTotals(*far).downlinksAcked);

  monitor.onFrame(makeRx(dataFrame(2, kFar, kBridge, 2), 5000000));
  monitor.tick(5100000);
  monitor.tick(8100000);
  TEST_ASSERT_EQUAL_UINT32(1, far->downlinksUnacked);  // No news in 3 s.

  monitor.notePeerAck(kFar, 3, false, 9000000);  // News that came first.
  monitor.onFrame(makeRx(dataFrame(3, kFar, kBridge, 3), 9001000));
  monitor.tick(9100000);
  TEST_ASSERT_EQUAL_UINT32(2, far->downlinksAcked);
  TEST_ASSERT_EQUAL_UINT32(2, far->ackedByPeer);

  monitor.setPeerLinked(false);  // Alone, a node decides at once.
  monitor.onFrame(makeRx(dataFrame(4, kFar, kBridge, 4), 10000000));
  monitor.tick(10100000);
  TEST_ASSERT_EQUAL_UINT32(2, far->downlinksUnacked);
}

void test_heard_answers_are_kept_for_the_other_node() {
  LinkMonitor monitor;
  monitor.setBridge(kBridge);
  HeardAck out[4];
  monitor.onFrame(makeRx(enhAck(1, kBridge, kFar), 1000000));
  TEST_ASSERT_EQUAL_size_t(0, monitor.takeHeardAcks(out, 4));  // Alone: nothing to share.
  monitor.setPeerLinked(true);
  monitor.onFrame(makeRx(enhAck(2, kBridge, kFar), 2000000));
  monitor.onFrame(makeRx(enhAck(3, kFar, kBridge), 2100000));
  TEST_ASSERT_EQUAL_size_t(2, monitor.takeHeardAcks(out, 4));
  TEST_ASSERT_EQUAL_MEMORY(kFar, out[0].device, kPrefixLen);
  TEST_ASSERT_EQUAL_UINT8(2, out[0].seq);
  TEST_ASSERT_FALSE(out[0].uplink);
  TEST_ASSERT_EQUAL_UINT8(3, out[1].seq);
  TEST_ASSERT_TRUE(out[1].uplink);
  TEST_ASSERT_EQUAL_size_t(0, monitor.takeHeardAcks(out, 4));
  for (uint8_t seq = 0; seq < 40; seq++) {
    monitor.onFrame(makeRx(enhAck(seq, kBridge, kFar), 3000000 + seq * 1000));
  }
  HeardAck many[kHeardAckCapacity + 4];
  TEST_ASSERT_EQUAL_size_t(kHeardAckCapacity, monitor.takeHeardAcks(many, kHeardAckCapacity + 4));
  TEST_ASSERT_EQUAL_UINT8(40 - kHeardAckCapacity, many[0].seq);  // The oldest went first.
}

void test_a_full_table_forgets_a_device_heard_once_long_ago() {
  LinkMonitor monitor;
  const auto hearFrom = [&monitor](uint8_t id, uint64_t atUs) {
    const uint8_t prefix[kPrefixLen] = {0xA1, 0, 0, 0, id};
    monitor.onFrame(makeRx(enhAck(1, kBridge, prefix), atUs));
  };
  for (size_t i = 0; i < kMaxDevices; i++) {
    hearFrom(static_cast<uint8_t>(i), 1000000);
  }
  hearFrom(200, 2000000);
  TEST_ASSERT_EQUAL_UINT32(1, monitor.counters().devicesNotTracked);
  hearFrom(201, 1000000 + 11 * kMinute);
  TEST_ASSERT_EQUAL_UINT32(1, monitor.counters().devicesNotTracked);
  const uint8_t newest[kPrefixLen] = {0xA1, 0, 0, 0, 201};
  TEST_ASSERT_NOT_NULL(monitor.findLink(newest));
  TEST_ASSERT_EQUAL_UINT32(1, monitor.findLink(newest)->frames);
  TEST_ASSERT_EQUAL_UINT64(0, monitor.hourTotals(*monitor.findLink(newest)).trainUs);
  const uint8_t first[kPrefixLen] = {0xA1, 0, 0, 0, 0};
  TEST_ASSERT_NULL(monitor.findLink(first));
}

void test_a_device_the_bridge_talks_to_is_never_forgotten() {
  LinkMonitor monitor;
  monitor.setBridge(kBridge);
  monitor.onFrame(makeRx(dataFrame(1, kFar, kBridge, 1), 1000000));
  monitor.onFrame(makeRx(dataFrame(2, kFar, kBridge, 2), 2000000));
  for (size_t i = 0; monitor.counters().devicesNotTracked == 0; i++) {
    const uint8_t prefix[kPrefixLen] = {0xA1, 0, 0, 0, static_cast<uint8_t>(i)};
    monitor.onFrame(makeRx(enhAck(1, kFar, prefix), 30 * kMinute));
  }
  TEST_ASSERT_NOT_NULL(monitor.findLink(kFar));
}

void test_a_device_proxied_toward_it_only_keeps_its_own_frames_measured() {
  LinkMonitor monitor;
  monitor.setBridge(kBridge);
  TEST_ASSERT_TRUE(monitor.setProxied(kFar, true, ProxyDirection::ToValve));
  TEST_ASSERT_TRUE(monitor.isProxied(kFar));
  TEST_ASSERT_FALSE(monitor.proxiesUplink(kFar));
  TEST_ASSERT_TRUE(monitor.proxiedDirection(0) == ProxyDirection::ToValve);
  monitor.setProxyActive(true, true);
  monitor.onFrame(makeRx(dataFrame(1, kBridge, kFar, 1), 1000000));
  monitor.onFrame(makeRx(enhAck(1, kFar, kBridge), 1006000));  // Only the bridge answers it.
  monitor.onFrame(makeRx(enhAck(2, kBridge, kFar), 2000000));  // May be the bridge node's.
  monitor.tick(2100000);
  TEST_ASSERT_EQUAL_UINT32(1, monitor.findLink(kFar)->uplinksAcked);
  TEST_ASSERT_EQUAL_UINT32(1, monitor.counters().verifiedAcks);
  TEST_ASSERT_TRUE(monitor.setProxied(kFar, true, ProxyDirection::Both));
  TEST_ASSERT_TRUE(monitor.proxiesUplink(kFar));
  TEST_ASSERT_EQUAL_size_t(1, monitor.proxiedCount());
}

void test_proxy_toward_the_valve_only_leaves_its_frames_to_the_bridge() {
  FakePort port;
  LinkMonitor monitor;
  monitor.setBridge(kBridge);
  monitor.setProxied(kFar, true, ProxyDirection::ToValve);
  ProxyEngine engine(port, monitor);
  engine.setProxyMode(true);
  engine.onFrame(makeRx(dataFrame(1, kBridge, kFar, 1), port.clock));
  runFor(engine, port, 50000);
  TEST_ASSERT_EQUAL_size_t(0, port.sent.size());
  engine.onFrame(makeRx(dataFrame(2, kFar, kBridge, 2), port.clock));
  runFor(engine, port, 50000);
  TEST_ASSERT_GREATER_OR_EQUAL(1, port.sent.size());
  EnhAck ack;
  TEST_ASSERT_TRUE(parseEnhAck(port.sent[0].data(), port.sent[0].size(), ack));
  TEST_ASSERT_EQUAL_UINT8(2, ack.seq);
  TEST_ASSERT_EQUAL_MEMORY(kBridge, ack.dstEui, kPrefixLen);
}

void test_a_device_proxied_toward_the_bridge_only_keeps_the_bridges_frames_measured() {
  LinkMonitor monitor;
  monitor.setBridge(kBridge);
  TEST_ASSERT_TRUE(monitor.setProxied(kFar, true, ProxyDirection::ToBridge));
  TEST_ASSERT_TRUE(monitor.proxiesUplink(kFar));
  TEST_ASSERT_FALSE(monitor.proxiesDownlink(kFar));
  TEST_ASSERT_FALSE(monitor.proxiesAnyDownlink());
  monitor.setProxyActive(true, true);
  monitor.onFrame(makeRx(dataFrame(1, kFar, kBridge, 1), 1000000));
  monitor.onFrame(makeRx(enhAck(1, kBridge, kFar), 1006000));  // Only the device answers the bridge.
  monitor.tick(2100000);
  TEST_ASSERT_EQUAL_UINT32(1, monitor.findLink(kFar)->downlinksAcked);
  TEST_ASSERT_EQUAL_UINT32(1, monitor.counters().verifiedAcks);
  TEST_ASSERT_TRUE(monitor.setProxied(kOther, true, ProxyDirection::Both));
  TEST_ASSERT_TRUE(monitor.proxiesAnyDownlink());
}

void test_proxy_toward_the_bridge_only_leaves_the_bridges_frames_to_the_valve() {
  FakePort port;
  LinkMonitor monitor;
  monitor.setBridge(kBridge);
  monitor.setProxied(kFar, true, ProxyDirection::ToBridge);
  ProxyEngine engine(port, monitor);
  engine.setProxyMode(true);
  engine.onFrame(makeRx(dataFrame(1, kFar, kBridge, 1), port.clock));
  runFor(engine, port, 50000);
  TEST_ASSERT_EQUAL_size_t(0, port.sent.size());
  engine.onFrame(makeRx(dataFrame(2, kBridge, kFar, 2), port.clock));
  runFor(engine, port, 50000);
  TEST_ASSERT_GREATER_OR_EQUAL(1, port.sent.size());
  EnhAck ack;
  TEST_ASSERT_TRUE(parseEnhAck(port.sent[0].data(), port.sent[0].size(), ack));
  TEST_ASSERT_EQUAL_UINT8(2, ack.seq);
  TEST_ASSERT_EQUAL_MEMORY(kFar, ack.dstEui, kPrefixLen);
}

void test_status_names_each_proxy_direction() {
  static char buffer[kStatusJsonCapacity];
  LinkMonitor monitor;
  monitor.setBridge(kBridge);
  monitor.setProxied(kFar, true, ProxyDirection::ToBridge);
  monitor.setProxied(kOther, true, ProxyDirection::Both);
  monitor.onFrame(makeRx(dataFrame(1, kFar, kBridge, 1), 1000000));
  monitor.onFrame(makeRx(dataFrame(2, kOther, kBridge, 2), 1000000));
  const std::string json = statusOf(monitor, nullptr, buffer, sizeof(buffer));
  for (const char* key : {"\"proxy_direction\":\"uplink\"", "\"proxy_direction\":\"both\""}) {
    TEST_ASSERT_NOT_NULL_MESSAGE(std::strstr(json.c_str(), key), key);
  }
}

void test_status_shows_windows_directions_and_trains() {
  static char buffer[kStatusJsonCapacity];
  LinkMonitor monitor;
  monitor.setBridge(kBridge);
  monitor.setProxied(kFar, true, ProxyDirection::ToValve);
  hearDownlink(monitor, 1, 1000000, 400000, true);
  const std::string json = statusOf(monitor, nullptr, buffer, sizeof(buffer));
  for (const char* key :
       {"\"proxy_direction\":\"downlink\"", "\"proxy_direction\":null", "\"long_train_ack_rate\":1.000",
        "\"acks_from_peer\":0", "\"already_answered\":null", "\"hour\":[0,0,0,1,1,0,0,1,0,",
        "\"day\":[0,0,0,1,1,0,0,1,0,", "\"total\":[0,0,0,1,1,0,0,1,0,0.00,0.00]",
        "\"windows\":{\"since_s\":5.0,\"hour_s\":5.0,\"day_s\":5.0}", "\"trains\":{\"hour\":[0.", "\"day\":[0."}) {
    TEST_ASSERT_NOT_NULL_MESSAGE(std::strstr(json.c_str(), key), key);
  }
}

// Every number at its widest, every device present and named, every frame kept.
void test_worst_case_status_with_everything_fits_the_firmware_buffer() {
  static char buffer[kStatusJsonCapacity];
  FakePort port;
  LinkMonitor monitor;
  NameTable names;
  monitor.setBridge(kBridge);
  for (int i = 0; i < 60; i++) {
    const uint8_t prefix[kPrefixLen] = {0xA1, 0, 0, 0, static_cast<uint8_t>(i)};
    monitor.onFrame(makeRx(dataFrame(static_cast<uint8_t>(i), prefix, kBridge, 1), 1000000));
    names.set(prefix, "123456789012345678901234");
    monitor.setProxied(prefix, true, i % 2 == 0 ? ProxyDirection::ToValve : ProxyDirection::Both);
  }
  for (size_t i = 0; i < kMaxDevices; i++) {
    DeviceLink& link = const_cast<DeviceLink&>(monitor.devices()[i]);
    for (uint32_t* count : {&link.frames, &link.uplinks, &link.uplinksAcked, &link.downlinks, &link.retransmissions,
                            &link.proxiedUplinks, &link.proxiedDownlinks, &link.deliveryFailures, &link.longTrainAcked,
                            &link.longTrainUnacked, &link.ackedByPeer, &link.alreadyAnswered}) {
      *count = 4000000000;
    }
    link.rssiDbm = -123.45f;
    link.airtime.record(DutyCycleLimiter::kBudgetUs, 1000000);
    for (LinkBucket* buckets : {link.hour, link.day}) {
      const size_t count = buckets == link.hour ? kHourBuckets : kDayBuckets;
      for (size_t b = 0; b < count; b++) {
        for (size_t f = 0; f < kLinkCountFieldCount; f++) {
          buckets[b].*kLinkCountFields[f] = UINT16_MAX;
        }
        buckets[b].airtimeUs = UINT32_MAX;
        buckets[b].trainUs = UINT32_MAX;
      }
    }
  }
  for (int i = 0; i < 30; i++) {
    monitor.onFrame(makeRx(enhAck(static_cast<uint8_t>(i), kBridge, kFar), 2000000 + i));
  }
  ProxyEngine engine(port, monitor);
  // A valve node that knows the bridge node's strength, so each device has a number for
  // the frames it already had, not null.
  engine.setRole(Role::Valve);
  engine.setProxyMode(true);
  const uint8_t first[kPrefixLen] = {0xA1, 0, 0, 0, 0};
  RxFrame peerAnswer = makeRx(enhAck(1, kBridge, first), 3000000);
  peerAnswer.rssiDbm = -50.5f;
  engine.onFrame(peerAnswer);
  TEST_ASSERT_TRUE(engine.canTellOwnAnswer(first));
  Survey survey;
  survey.start(0, 1);
  uint64_t now = 0;
  while (survey.running()) {
    survey.onFrame(true, 0);
    survey.tick(now += 2);
  }
  PairStatus pair = {};
  pair.enabled = true;
  pair.hasPeer = true;
  pair.hasOwnRssi = true;
  pair.ownRssi = -123.5f;
  pair.hasPeerRssi = true;
  pair.peerRssi = -123.5f;
  pair.peerRestarts = 4000000000;
  pair.rejected = 4000000000;
  StatusInput input = {};
  input.monitor = &monitor;
  input.engine = &engine;
  input.survey = &survey;
  input.names = &names;
  input.pair = &pair;
  input.radio = {true, 0x14, 26, -13, 4000000000, 4000000000};
  input.firmware = "123.456.789";
  input.variant = "esp32s3octal";
  input.freeHeap = 4000000000;
  input.minFreeHeap = 4000000000;
  input.nowUs = 4000000000ULL * 1000000ULL;
  input.startedUs = 0;
  TEST_ASSERT_TRUE(writeStatusJson(input, buffer, sizeof(buffer)));
  TEST_ASSERT_NOT_NULL(std::strstr(buffer, "\"already_answered\":4000000000"));
  TEST_ASSERT_NOT_NULL(std::strstr(buffer, "\"peer_answer_rssi_dbm\":-50.5"));
  TEST_ASSERT_LESS_THAN_size_t(kStatusJsonCapacity * 95 / 100, std::strlen(buffer));  // Some room to spare.
}

void test_answers_heard_by_one_node_count_on_both() {
  Node a(1, 100);
  Node b(2, 200);
  pairUp(a, b);
  TEST_ASSERT_TRUE(a.monitor.peerLinked());
  b.monitor.onFrame(makeRx(dataFrame(5, kFar, kBridge, 50), b.port.clock));
  b.monitor.tick(b.port.clock + 30000);
  TEST_ASSERT_EQUAL_UINT32(0, b.monitor.findLink(kFar)->downlinksAcked);
  a.monitor.onFrame(makeRx(enhAck(5, kBridge, kFar), a.port.clock));
  advance(a, b, 1000000);
  TEST_ASSERT_EQUAL_UINT32(1, b.monitor.findLink(kFar)->downlinksAcked);
  TEST_ASSERT_EQUAL_UINT32(1, b.monitor.findLink(kFar)->ackedByPeer);
  TEST_ASSERT_EQUAL_UINT32(0, b.pair.rejected());
}

void test_names_travel_to_the_other_node() {
  Node a(1, 100);
  Node b(2, 200);
  NameTable namesA;
  NameTable namesB;
  const uint8_t kNear[kPrefixLen] = {0xA0, 0, 0, 0, 3};
  namesA.set(kFar, "Werkkamer");
  namesA.set(kNear, "Keuken");
  namesB.set(kNear, "Kitchen");
  a.pair.setNames(&namesA);
  b.pair.setNames(&namesB);
  pairUp(a, b);
  advance(a, b, 5000000);
  TEST_ASSERT_EQUAL_STRING("Werkkamer", namesB.find(kFar));
  TEST_ASSERT_EQUAL_STRING("Kitchen", namesB.find(kNear));  // Each keeps its own.
  TEST_ASSERT_EQUAL_STRING("Keuken", namesA.find(kNear));
  TEST_ASSERT_TRUE(b.pair.takeNamesChanged());
  TEST_ASSERT_FALSE(b.pair.takeNamesChanged());
  namesA.set(kNear, "Bijkeuken");
  a.pair.sendName(kNear, "Bijkeuken");  // What the user sets replaces it.
  advance(a, b, 500000);
  TEST_ASSERT_EQUAL_STRING("Bijkeuken", namesB.find(kNear));
  namesA.set(kFar, "");  // As the web server does: the own table first, then the other node.
  a.pair.sendName(kFar, "");
  advance(a, b, 500000);
  TEST_ASSERT_NULL(namesB.find(kFar));
  TEST_ASSERT_TRUE(b.pair.takeNamesChanged());
  TEST_ASSERT_EQUAL_UINT32(0, b.pair.rejected());
}

void test_a_name_lost_on_the_way_arrives_later() {
  Node a(1, 100);
  Node b(2, 200);
  NameTable namesA;
  NameTable namesB;
  a.pair.setNames(&namesA);
  b.pair.setNames(&namesB);
  pairUp(a, b);
  advance(a, b, 10000000);        // The first round is over, with nothing to send.
  namesA.set(kFar, "Werkkamer");  // Its message lost: only the table has it.
  advance(a, b, 70000000);        // Well past a round of one name per Hello.
  TEST_ASSERT_EQUAL_STRING("Werkkamer", namesB.find(kFar));
  TEST_ASSERT_TRUE(b.pair.takeNamesChanged());
  advance(a, b, 70000000);
  TEST_ASSERT_FALSE(b.pair.takeNamesChanged());  // Sent again, but nothing changed.
  TEST_ASSERT_EQUAL_UINT32(0, b.pair.rejected());
}

void test_a_bad_name_from_the_other_node_is_refused() {
  Node a(1, 100);
  Node b(2, 200);
  NameTable namesB;
  b.pair.setNames(&namesB);
  pairUp(a, b);
  a.pair.sendName(kFar, "say \"hi\"");
  advance(a, b, 500000);
  TEST_ASSERT_NULL(namesB.find(kFar));
  TEST_ASSERT_EQUAL_UINT32(1, b.pair.rejected());
}

void test_proxy_direction_travels_to_the_partner() {
  Node a(1, 100);
  Node b(2, 200);
  pairUp(a, b);
  a.monitor.setProxied(kFar, true, ProxyDirection::ToValve);
  a.pair.syncProxied();
  advance(a, b, 1000000);
  TEST_ASSERT_TRUE(b.monitor.isProxied(kFar));
  TEST_ASSERT_FALSE(b.monitor.proxiesUplink(kFar));
  TEST_ASSERT_TRUE(b.pair.takeProxiedChanged());
  a.monitor.setProxied(kFar, true, ProxyDirection::Both);
  a.pair.syncProxied();
  advance(a, b, 1000000);
  TEST_ASSERT_TRUE(b.monitor.proxiesUplink(kFar));
  TEST_ASSERT_TRUE(b.pair.takeProxiedChanged());
  a.monitor.setProxied(kFar, true, ProxyDirection::ToBridge);
  a.pair.syncProxied();
  advance(a, b, 1000000);
  TEST_ASSERT_TRUE(b.monitor.proxiesUplink(kFar));
  TEST_ASSERT_FALSE(b.monitor.proxiesDownlink(kFar));
  TEST_ASSERT_TRUE(b.pair.takeProxiedChanged());
  TEST_ASSERT_EQUAL_UINT32(0, b.pair.rejected());
}

void test_an_unknown_proxy_direction_changes_nothing() {
  Node a(1, 100);
  Node b(2, 200);
  pairUp(a, b);
  b.monitor.setProxied(kFar, true, ProxyDirection::ToValve);
  a.monitor.setProxied(kFar, true, static_cast<ProxyDirection>(3));  // As a later version might send.
  a.pair.syncProxied();
  advance(a, b, 1000000);
  TEST_ASSERT_TRUE(b.monitor.proxiedDirection(0) == ProxyDirection::ToValve);
  TEST_ASSERT_FALSE(b.pair.takeProxiedChanged());
  TEST_ASSERT_EQUAL_UINT32(1, b.pair.rejected());
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_builders_match_the_python_vectors);
  RUN_TEST(test_duty_budget_blocks_and_returns);
  RUN_TEST(test_slot_is_centred_on_a_sample_and_in_the_future);
  RUN_TEST(test_large_phase_wraps_on_the_period);
  RUN_TEST(test_old_or_missed_anchor_gives_no_slot);
  RUN_TEST(test_anchor_phase_round_trip);
  RUN_TEST(test_engine_starts_in_monitor_and_never_transmits);
  RUN_TEST(test_proxy_acks_then_forwards_byte_for_byte);
  RUN_TEST(test_unnamed_device_is_left_alone);
  RUN_TEST(test_late_ack_is_dropped_and_counted);
  RUN_TEST(test_forward_is_retried_then_abandoned_and_reported);
  RUN_TEST(test_direct_bridge_ack_cancels_the_forward);
  RUN_TEST(test_full_queue_withholds_the_ack);
  RUN_TEST(test_leaving_proxy_mode_drops_pending_work);
  RUN_TEST(test_transmitter_stops_at_the_budget);
  RUN_TEST(test_accepted_work_is_reserved_from_the_budget);
  RUN_TEST(test_proxied_list_is_bounded);
  RUN_TEST(test_status_reports_the_proxy);
  RUN_TEST(test_names_are_set_replaced_and_cleared);
  RUN_TEST(test_names_that_json_cannot_carry_are_refused);
  RUN_TEST(test_full_name_table_refuses_a_new_device);
  RUN_TEST(test_status_shows_the_name_and_the_last_downlink);
  RUN_TEST(test_worst_case_status_with_names_fits);
  RUN_TEST(test_host_names_of_the_node_are_allowed);
  RUN_TEST(test_other_host_names_are_refused);
  RUN_TEST(test_wrong_tokens_wait_longer_each_time);
  RUN_TEST(test_one_client_cannot_lock_out_another);
  RUN_TEST(test_sha256_known_answers);
  RUN_TEST(test_hmac_known_answer);
  RUN_TEST(test_pbkdf2_known_answers);
  RUN_TEST(test_pair_key_is_derived_from_the_token);
  RUN_TEST(test_constant_time_compare);
  RUN_TEST(test_pair_datagram_round_trip);
  RUN_TEST(test_forged_datagrams_are_refused);
  RUN_TEST(test_strongest_node_becomes_the_bridge_node);
  RUN_TEST(test_a_node_that_hears_no_bridge_becomes_the_valve_node);
  RUN_TEST(test_no_roles_while_neither_hears_the_bridge);
  RUN_TEST(test_roles_do_not_flip_on_a_few_db);
  RUN_TEST(test_nodes_stay_in_monitor_until_asked);
  RUN_TEST(test_one_request_starts_both_nodes_with_their_roles);
  RUN_TEST(test_one_request_stops_both_nodes);
  RUN_TEST(test_no_transmission_before_the_partner_listens);
  RUN_TEST(test_start_gives_up_if_the_partner_never_answers);
  RUN_TEST(test_lost_link_pauses_and_a_returning_link_resumes);
  RUN_TEST(test_a_restart_of_one_node_stops_the_other);
  RUN_TEST(test_replayed_frame_is_refused);
  RUN_TEST(test_frame_for_an_earlier_boot_is_refused);
  RUN_TEST(test_replayed_hellos_of_an_older_boot_cannot_restart_a_pair);
  RUN_TEST(test_a_third_node_is_ignored);
  RUN_TEST(test_proxied_list_travels_to_the_partner);
  RUN_TEST(test_a_node_without_a_partner_proxies_alone);
  RUN_TEST(test_pairing_off_without_a_token);
  RUN_TEST(test_frame_goes_from_valve_node_to_bridge_node_and_on_air);
  RUN_TEST(test_status_shows_the_pair);
  RUN_TEST(test_json_fields_for_the_web_api);
  RUN_TEST(test_fcs_is_crc16_kermit);
  RUN_TEST(test_vectors_classify_like_python);
  RUN_TEST(test_vectors_parse_like_python);
  RUN_TEST(test_retransmission_hash_groups_retries);
  RUN_TEST(test_airtime_of_a_wake_frame);
  RUN_TEST(test_phy_matches_the_tado_air_interface);
  RUN_TEST(test_dedup_sorts_new_echo_and_retry);
  RUN_TEST(test_dedup_window_expires);
  RUN_TEST(test_dedup_table_survives_more_frames_than_it_holds);
  RUN_TEST(test_frame_arriving_byte_by_byte_is_whole);
  RUN_TEST(test_frame_longer_than_the_fifo_is_whole);
  RUN_TEST(test_fifo_is_never_emptied_in_mid_packet);
  RUN_TEST(test_two_frames_in_a_row);
  RUN_TEST(test_status_bytes_are_decoded);
  RUN_TEST(test_impossible_length_flushes);
  RUN_TEST(test_overflow_recovers);
  RUN_TEST(test_stalled_frame_is_abandoned);
  RUN_TEST(test_bridge_is_learned_from_two_trains);
  RUN_TEST(test_a_forged_train_from_another_sender_does_not_name_the_bridge);
  RUN_TEST(test_data_without_a_train_teaches_nothing);
  RUN_TEST(test_acked_uplink_counts_as_good);
  RUN_TEST(test_unanswered_uplink_counts_as_bad);
  RUN_TEST(test_downlink_ack_is_tracked);
  RUN_TEST(test_crc_failure_is_only_counted);
  RUN_TEST(test_bad_fcs_is_counted_and_not_verified);
  RUN_TEST(test_device_table_full_is_counted_not_fatal);
  RUN_TEST(test_recent_frames_are_newest_first_and_bounded);
  RUN_TEST(test_trim_adds_the_median_offset);
  RUN_TEST(test_trim_ignores_clipped_readings);
  RUN_TEST(test_trim_rounds_like_python);
  RUN_TEST(test_no_usable_reading_gives_no_trim);
  RUN_TEST(test_survey_visits_every_step_and_finds_the_best);
  RUN_TEST(test_silent_survey_has_no_best);
  RUN_TEST(test_status_has_every_field_the_dashboard_reads);
  RUN_TEST(test_worst_case_status_fits_the_firmware_buffer);
  RUN_TEST(test_status_reports_a_buffer_that_is_too_small);
  RUN_TEST(test_json_int_field);
  RUN_TEST(test_image_for_this_board_is_accepted_in_pieces_of_any_size);
  RUN_TEST(test_marker_is_found_among_text_that_looks_like_it);
  RUN_TEST(test_image_with_another_projects_marker_is_refused);
  RUN_TEST(test_wrong_images_are_refused);
  RUN_TEST(test_airtime_of_an_hour_is_dropped_a_bucket_late);
  RUN_TEST(test_acks_of_other_devices_are_still_measured_in_a_pair);
  RUN_TEST(test_handed_over_frames_are_known_for_as_long_as_the_other_node_tries);
  RUN_TEST(test_reset_button_acts_when_a_long_press_is_released);
  RUN_TEST(test_reset_button_ignores_a_press_of_the_wrong_length);
  RUN_TEST(test_reset_button_ignores_a_press_after_the_window);
  RUN_TEST(test_reset_button_waits_for_a_press_that_began_in_the_window);
  RUN_TEST(test_windows_keep_the_last_hour_and_the_last_day);
  RUN_TEST(test_windows_start_again_when_the_proxy_starts_or_stops);
  RUN_TEST(test_downlinks_behind_long_and_short_trains_are_told_apart);
  RUN_TEST(test_wakeup_trains_are_counted_by_sender);
  RUN_TEST(test_a_train_for_another_device_is_not_claimed);
  RUN_TEST(test_an_answer_heard_only_by_the_other_node_counts);
  RUN_TEST(test_heard_answers_are_kept_for_the_other_node);
  RUN_TEST(test_a_full_table_forgets_a_device_heard_once_long_ago);
  RUN_TEST(test_a_device_the_bridge_talks_to_is_never_forgotten);
  RUN_TEST(test_a_device_proxied_toward_it_only_keeps_its_own_frames_measured);
  RUN_TEST(test_proxy_toward_the_valve_only_leaves_its_frames_to_the_bridge);
  RUN_TEST(test_a_device_proxied_toward_the_bridge_only_keeps_the_bridges_frames_measured);
  RUN_TEST(test_proxy_toward_the_bridge_only_leaves_the_bridges_frames_to_the_valve);
  RUN_TEST(test_status_names_each_proxy_direction);
  RUN_TEST(test_status_shows_windows_directions_and_trains);
  RUN_TEST(test_worst_case_status_with_everything_fits_the_firmware_buffer);
  RUN_TEST(test_answers_heard_by_one_node_count_on_both);
  RUN_TEST(test_names_travel_to_the_other_node);
  RUN_TEST(test_a_name_lost_on_the_way_arrives_later);
  RUN_TEST(test_a_bad_name_from_the_other_node_is_refused);
  RUN_TEST(test_proxy_direction_travels_to_the_partner);
  RUN_TEST(test_an_unknown_proxy_direction_changes_nothing);
  return UNITY_END();
}
