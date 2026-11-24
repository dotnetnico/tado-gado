#include "radio_task.h"

#include <Arduino.h>
#include <WiFi.h>
#include <esp_task_wdt.h>
#include <lwip/sockets.h>

#include <atomic>
#include <cstring>
#include <new>

#include "board.h"
#include "build_info.h"
#include "cc1101.h"
#include "link_monitor.h"
#include "pair_manager.h"
#include "proxy_engine.h"
#include "rx_assembler.h"
#include "status_json.h"
#include "survey.h"
#include "update.h"

namespace {

constexpr BaseType_t kRadioCore = 1;  // Wi-Fi runs on core 0.
constexpr UBaseType_t kRadioPriority = 5;
constexpr uint32_t kStackBytes = 8192;
constexpr uint64_t kSurveyDwellUs = 20 * 1000000ULL;
constexpr uint64_t kHealthCheckUs = 10 * 1000000ULL;
// The Tado register set has no automatic calibration, so the host repeats it.
constexpr uint64_t kRecalibrateUs = 30 * 60 * 1000000ULL;
constexpr TickType_t kMissingChipDelay = pdMS_TO_TICKS(1000);
// Each pair datagram costs an HMAC. A flood on the port must not keep the radio waiting.
constexpr int kMaxDatagramsPerPass = 4;

SemaphoreHandle_t lock;

Cc1101 chip;
tadogado::RxAssembler assembler(chip);
tadogado::LinkMonitor monitor;
tadogado::Survey survey;
tadogado::NameTable names;
TaskHandle_t task;

// Connects the engine to the chip. The lock is free during a transmission, so the
// dashboard can answer during the train of a sleeping device.
class ChipPort : public tadogado::RadioPort {
 public:
  bool send(const uint8_t* frame, size_t length) override {
    xSemaphoreGive(lock);
    const bool ok = chip.send(frame, length);
    xSemaphoreTake(lock, portMAX_DELAY);
    assembler.reset();
    return ok;
  }

  void sleepUntil(uint64_t timeUs) override {
    while (static_cast<uint64_t>(esp_timer_get_time()) < timeUs) {
    }
  }

  uint64_t nowUs() override { return esp_timer_get_time(); }
};

ChipPort port;
tadogado::ProxyEngine engine(port, monitor);

// The link to the other node of a pair, over UDP on the home network. A plain socket
// rather than WiFiUDP: the radio task polls it every millisecond, and WiFiUDP takes a
// 1460-byte buffer from the heap on each poll.
class UdpTransport : public tadogado::PairTransport {
 public:
  void begin() {
    socket_ = lwip_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (socket_ < 0) {
      return;
    }
    int enabled = 1;
    lwip_setsockopt(socket_, SOL_SOCKET, SO_REUSEADDR, &enabled, sizeof(enabled));
    lwip_setsockopt(socket_, SOL_SOCKET, SO_BROADCAST, &enabled, sizeof(enabled));
    sockaddr_in local = {};
    local.sin_family = AF_INET;
    local.sin_port = htons(tadogado::kPairPort);
    local.sin_addr.s_addr = htonl(INADDR_ANY);
    if (lwip_bind(socket_, reinterpret_cast<sockaddr*>(&local), sizeof(local)) != 0) {
      lwip_close(socket_);
      socket_ = -1;
    }
  }

  void broadcast(const uint8_t* data, size_t length) override {
    if (WiFi.status() == WL_CONNECTED) {
      sendTo(static_cast<uint32_t>(WiFi.broadcastIP()), data, length);
    }
  }

  void sendToPeer(const uint8_t* data, size_t length) override {
    if (peer_ != 0 && WiFi.status() == WL_CONNECTED) {
      sendTo(peer_, data, length);
    }
  }

  void rememberPeer() override { peer_ = lastSender_; }

  // Returns the length of the next datagram, or 0. Never blocks.
  size_t receive(uint8_t* out, size_t capacity) {
    if (socket_ < 0) {
      return 0;
    }
    sockaddr_in sender = {};
    socklen_t senderLength = sizeof(sender);
    const int length =
        lwip_recvfrom(socket_, out, capacity, MSG_DONTWAIT, reinterpret_cast<sockaddr*>(&sender), &senderLength);
    if (length <= 0) {
      return 0;
    }
    lastSender_ = sender.sin_addr.s_addr;
    return static_cast<size_t>(length);
  }

 private:
  void sendTo(uint32_t address, const uint8_t* data, size_t length) {
    if (socket_ < 0) {
      return;
    }
    sockaddr_in target = {};
    target.sin_family = AF_INET;
    target.sin_port = htons(tadogado::kPairPort);
    target.sin_addr.s_addr = address;
    lwip_sendto(socket_, data, length, 0, reinterpret_cast<sockaddr*>(&target), sizeof(target));
  }

  int socket_ = -1;
  uint32_t peer_ = 0;  // Network byte order, as in sockaddr_in.
  uint32_t lastSender_ = 0;
};

UdpTransport transport;
tadogado::PairManager pair(transport, engine, monitor);

tadogado::RadioStatus radioStatus;
uint8_t outputPower;
uint64_t startedUs;

// Requests from the web task. The radio task picks them up in its loop, between
// jobs, so a mode change can never land in the middle of a transaction.
std::atomic<bool> retuneRequested{false};
std::atomic<uint8_t> requestedChannel{0};
std::atomic<int8_t> requestedTrim{0};
std::atomic<bool> powerRequested{false};
std::atomic<uint8_t> requestedPower{0};
std::atomic<bool> surveyRequested{false};
std::atomic<bool> modeProxy{false};

void IRAM_ATTR onGdoRise() {
  BaseType_t woken = pdFALSE;
  vTaskNotifyGiveFromISR(task, &woken);
  if (woken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}

void bringUpChip() {
  const bool found = chip.probe();
  radioStatus.found = found;
  radioStatus.version = found ? chip.version() : 0;
  if (found) {
    chip.configure(radioStatus.channel, radioStatus.fsctrl0, outputPower);
    assembler.reset();
  }
}

// A survey leaves the radio on another channel or trim, so nothing may be sent then.
void stopSurvey() {
  if (survey.running()) {
    survey.stop();
    chip.retune(radioStatus.channel, radioStatus.fsctrl0);
    assembler.reset();
  }
}

void servicePair(uint64_t nowUs) {
  static uint8_t datagram[tadogado::kPairMaxDatagram];
  for (int i = 0; i < kMaxDatagramsPerPass; i++) {
    const size_t length = transport.receive(datagram, sizeof(datagram));
    if (length == 0) {
      break;
    }
    pair.onDatagram(datagram, length, nowUs);
  }
  pair.tick(nowUs);
  modeProxy = pair.wanted();
  // The other node can start proxying while this one runs a survey.
  if (pair.wanted() || engine.proxyMode()) {
    stopSurvey();
  }
}

void serviceRequests(uint64_t nowUs) {
  // A retune or calibration flushes the FIFO and takes milliseconds, so it waits until
  // no ACK is due and no job is under way. The request stays pending until then.
  if (!engine.idle()) {
    return;
  }
  if (retuneRequested.exchange(false)) {
    radioStatus.channel = requestedChannel.load();
    radioStatus.fsctrl0 = requestedTrim.load();
    survey.stop();
    chip.retune(radioStatus.channel, radioStatus.fsctrl0);
    assembler.reset();
  }
  if (powerRequested.exchange(false)) {
    outputPower = requestedPower.load();
    chip.setOutputPower(outputPower);
  }
  if (surveyRequested.exchange(false) && !pair.wanted()) {
    survey.start(nowUs, kSurveyDwellUs);
  }
}

void serviceSurvey(uint64_t nowUs) {
  const bool wasRunning = survey.running();
  if (survey.tick(nowUs)) {
    chip.retune(survey.current().channel, survey.current().fsctrl0);
    assembler.reset();
  } else if (wasRunning && !survey.running()) {
    // Finished: back to the settings in use until the user applies a new one.
    chip.retune(radioStatus.channel, radioStatus.fsctrl0);
    assembler.reset();
  }
}

void radioLoop(void*) {
  // A radio task that hangs restarts the node, and so returns it to monitor mode: the
  // longest legitimate pass is a one-second wake-up train, the timeout five seconds.
  esp_task_wdt_add(nullptr);
  chip.begin();
  xSemaphoreTake(lock, portMAX_DELAY);
  bringUpChip();

  uint64_t nextHealthCheck = esp_timer_get_time() + kHealthCheckUs;
  uint64_t nextCalibration = esp_timer_get_time() + kRecalibrateUs;
  tadogado::RxFrame frame;

  for (;;) {
    esp_task_wdt_reset();
    if (!radioStatus.found) {
      xSemaphoreGive(lock);
      vTaskDelay(kMissingChipDelay);
      xSemaphoreTake(lock, portMAX_DELAY);
      bringUpChip();
      continue;
    }

    // A byte takes 160 us and the FIFO holds 64, so 1 ms is an easy margin, and the
    // GDO interrupts wake this task at once for the 5 ms ACK deadline.
    xSemaphoreGive(lock);
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1));
    xSemaphoreTake(lock, portMAX_DELAY);

    const uint64_t now = esp_timer_get_time();
    serviceRequests(now);
    serviceSurvey(now);
    servicePair(now);
    while (assembler.poll(esp_timer_get_time(), frame)) {
      if (survey.running()) {
        survey.onFrame(frame.crcOk, chip.frequencyError());
      }
      engine.onFrame(frame);
    }
    engine.tick();
    radioStatus.overflows = assembler.overflows;

    if (chip.takeConfigurationLost()) {
      radioStatus.resets++;
      bringUpChip();
    }
    if (now >= nextHealthCheck) {
      nextHealthCheck = now + kHealthCheckUs;
      if (!chip.configurationIntact()) {
        radioStatus.resets++;
        bringUpChip();
      }
    }
    if (now >= nextCalibration && !assembler.busy() && engine.idle()) {
      nextCalibration = now + kRecalibrateUs;
      chip.calibrate();
    }
  }
}

}  // namespace

namespace radio {

void start(const Config& config) {
  lock = xSemaphoreCreateMutex();
  startedUs = esp_timer_get_time();
  radioStatus.channel = config.channel;
  radioStatus.fsctrl0 = config.fsctrl0;
  outputPower = config.pa;
  for (size_t i = 0; i < config.proxiedCount; i++) {
    // The flash contents are not trusted: an unknown direction serves both ways.
    const uint8_t direction =
        config.proxiedDirection[i] <= tadogado::kProxyDirectionLast ? config.proxiedDirection[i] : 0;
    monitor.setProxied(config.proxied + i * tadogado::kPrefixLen, true,
                       static_cast<tadogado::ProxyDirection>(direction));
  }
  for (size_t i = 0; i < tadogado::NameTable::kCapacity; i++) {
    tadogado::NameTable::Entry entry = config.names[i];
    entry.name[tadogado::kMaxNameBytes] = '\0';  // The flash contents are not trusted.
    if (entry.used) {
      names.set(entry.prefix, entry.name);
    }
  }
  engine.setPeer(&pair);
  pair.setNames(&names);
  // Pulled down, so a missing chip cannot leave the pins floating into an interrupt storm.
  pinMode(board::kPinGdo0, INPUT_PULLDOWN);
  pinMode(board::kPinGdo2, INPUT_PULLDOWN);
  xTaskCreatePinnedToCore(radioLoop, "radio", kStackBytes, nullptr, kRadioPriority, &task, kRadioCore);
  attachInterrupt(digitalPinToInterrupt(board::kPinGdo0), onGdoRise, RISING);
  attachInterrupt(digitalPinToInterrupt(board::kPinGdo2), onGdoRise, RISING);
}

void startPairing(const char* token, const uint8_t* nodeId, uint32_t bootNumber) {
  // Slow on purpose, so it runs before taking the lock.
  uint8_t key[tadogado::kPairKeyLen] = {};
  const bool hasToken = token[0] != '\0';
  if (hasToken) {
    tadogado::derivePairKey(token, key);
  }
  xSemaphoreTake(lock, portMAX_DELAY);
  transport.begin();
  pair.begin(key, hasToken ? sizeof(key) : 0, nodeId, bootNumber);
  xSemaphoreGive(lock);
}

// Only the task of the web server calls this. It copies the state under the lock, which takes tens
// of microseconds, and formats the JSON after releasing it, which takes milliseconds:
// the radio task must not wait that long with an ACK due.
bool writeStatus(char* out, size_t capacity, bool readOnly) {
  static tadogado::LinkMonitor monitorCopy;
  static tadogado::Survey surveyCopy;
  static tadogado::NameTable namesCopy;
  alignas(tadogado::ProxyEngine) static uint8_t engineStorage[sizeof(tadogado::ProxyEngine)];
  tadogado::PairStatus pairCopy;
  tadogado::StatusInput input = {};

  xSemaphoreTake(lock, portMAX_DELAY);
  monitorCopy = monitor;
  surveyCopy = survey;
  namesCopy = names;
  // The copy keeps references to the port and monitor of the original, but the JSON
  // reads only its own fields through const getters.
  const tadogado::ProxyEngine* engineCopy = new (engineStorage) tadogado::ProxyEngine(engine);
  pairCopy = pair.status(esp_timer_get_time());
  input.radio = radioStatus;
  input.nowUs = esp_timer_get_time();
  xSemaphoreGive(lock);

  input.monitor = &monitorCopy;
  input.engine = engineCopy;
  input.survey = &surveyCopy;
  input.names = &namesCopy;
  input.pair = &pairCopy;  // writePair shows null when pairing is off.
  input.firmware = firmwareVersion();
  input.variant = kVariant;
  input.unconfirmed = update::unconfirmed();
  input.readOnly = readOnly;
  input.freeHeap = ESP.getFreeHeap();
  input.minFreeHeap = ESP.getMinFreeHeap();
  input.startedUs = startedUs;
  const bool ok = tadogado::writeStatusJson(input, out, capacity);
  engineCopy->~ProxyEngine();
  return ok;
}

bool idle() {
  xSemaphoreTake(lock, portMAX_DELAY);
  const bool quiet = engine.idle();
  xSemaphoreGive(lock);
  return quiet;
}

void requestRetune(uint8_t channel, int8_t fsctrl0) {
  requestedChannel = channel;
  requestedTrim = fsctrl0;
  retuneRequested = true;
}

void requestPower(uint8_t pa) {
  requestedPower = pa;
  powerRequested = true;
}

bool requestProxyMode(bool enabled) {
  xSemaphoreTake(lock, portMAX_DELAY);
  // A survey stops on the radio task before the proxy sends anything: see servicePair.
  const bool ok = pair.requestProxy(enabled, esp_timer_get_time());
  xSemaphoreGive(lock);
  return ok;
}

bool proxyMode() { return modeProxy.load(); }

bool requestSurvey() {
  if (proxyMode()) {
    return false;
  }
  surveyRequested = true;
  return true;
}

bool setProxied(const uint8_t* prefix, bool enabled, tadogado::ProxyDirection direction) {
  xSemaphoreTake(lock, portMAX_DELAY);
  const bool ok = monitor.setProxied(prefix, enabled, direction);
  if (ok) {
    pair.syncProxied();
  }
  xSemaphoreGive(lock);
  return ok;
}

bool takeProxiedChanged() {
  xSemaphoreTake(lock, portMAX_DELAY);
  const bool changed = pair.takeProxiedChanged();
  xSemaphoreGive(lock);
  return changed;
}

size_t proxiedDevices(uint8_t* out, uint8_t* directions) {
  xSemaphoreTake(lock, portMAX_DELAY);
  const size_t count = monitor.proxiedCount();
  for (size_t i = 0; i < count; i++) {
    std::memcpy(out + i * tadogado::kPrefixLen, monitor.proxied(i), tadogado::kPrefixLen);
    directions[i] = static_cast<uint8_t>(monitor.proxiedDirection(i));
  }
  xSemaphoreGive(lock);
  return count;
}

bool setName(const uint8_t* prefix, const char* name) {
  xSemaphoreTake(lock, portMAX_DELAY);
  const bool ok = names.set(prefix, name);
  if (ok) {
    pair.sendName(prefix, name);
  }
  xSemaphoreGive(lock);
  return ok;
}

bool takeNamesChanged() {
  xSemaphoreTake(lock, portMAX_DELAY);
  const bool changed = pair.takeNamesChanged();
  xSemaphoreGive(lock);
  return changed;
}

void copyNames(tadogado::NameTable::Entry* out) {
  xSemaphoreTake(lock, portMAX_DELAY);
  std::memcpy(out, names.entries(), sizeof(tadogado::NameTable::Entry) * tadogado::NameTable::kCapacity);
  xSemaphoreGive(lock);
}

}  // namespace radio
