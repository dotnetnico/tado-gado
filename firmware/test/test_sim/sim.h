// A small discrete-time model of a Tado network, for testing the proxy engine.
//
// It holds a bridge, sleeping valves and any number of proxy nodes on one channel,
// with a table of who can hear whom. A frame arrives only at a node that listened
// for all of it, with no other audible frame overlapping it. The Tado behaviour here
// comes from the published captures, with one assumption: the valve's ACK wait.
//
// This shows that the proxy logic fits the protocol as documented. It proves nothing
// about real RF or about the radio driver.
#pragma once

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>
#include <initializer_list>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "frames.h"
#include "link_monitor.h"
#include "proxy_engine.h"
#include "rx_assembler.h"

namespace sim {

using Bytes = std::vector<uint8_t>;
using tadogado::FrameClass;

constexpr int64_t kTickUs = 100;
constexpr int64_t kPeriodUs = static_cast<int64_t>(tadogado::kCslPeriodUs);
constexpr int64_t kUnitUs = static_cast<int64_t>(tadogado::kCslUnitUs);
constexpr int64_t kWakeStepUs = static_cast<int64_t>(tadogado::kWakeStepUs);
// Assumption: nothing published gives the valve's ACK timeout.
constexpr int64_t kValveAckWaitUs = 15000;
constexpr int64_t kSampleWindowUs = 8000;
constexpr int64_t kRendezvousWindowUs = 40000;
constexpr int64_t kAckDelayUs = 6000;
constexpr int64_t kRetryDelayUs = 2000000;

inline int64_t seconds(double value) { return static_cast<int64_t>(value * 1e6 + 0.5); }

inline int64_t modPeriod(int64_t value) { return ((value % kPeriodUs) + kPeriodUs) % kPeriodUs; }

inline const Bytes kEuiTail = {0xc5, 0x1b, 0x00};

inline Bytes eui(std::initializer_list<uint8_t> prefix) {
  Bytes out(prefix);
  out.insert(out.end(), kEuiTail.begin(), kEuiTail.end());
  return out;
}

inline const Bytes kBridgeEui = eui({0xb0, 0x00, 0x00, 0x00, 0x01});
inline const Bytes kFarEui = eui({0xa0, 0x00, 0x00, 0x00, 0x02});
inline const Bytes kNearEui = eui({0xa0, 0x00, 0x00, 0x00, 0x03});

inline Bytes prefixOf(const Bytes& eui) { return Bytes(eui.begin(), eui.begin() + tadogado::kPrefixLen); }

// The short address of a device, as a wake-up frame carries it.
inline Bytes shortAddress(const Bytes& eui) { return Bytes{eui[1], eui[0]}; }

inline int64_t floorDiv(int64_t value, int64_t divisor) {
  const int64_t quotient = value / divisor;
  return (value % divisor != 0 && value < 0) ? quotient - 1 : quotient;
}

inline void appendLe(Bytes& out, uint32_t value, int bytes) {
  for (int i = 0; i < bytes; i++) {
    out.push_back(static_cast<uint8_t>(value >> (8 * i)));
  }
}

inline Bytes bytesOf(const char* text) { return Bytes(text, text + std::strlen(text)); }

// A Data frame shaped as on air, with a stand-in for the cipher. A real retry reuses
// the nonce, so only the counter and FCS positions and the MIC change. This keeps
// that property without doing any cryptography.
inline Bytes dataFrame(uint8_t seq, const Bytes& dst, const Bytes& src, uint32_t counter, const Bytes& datagram,
                       bool pending = false) {
  Bytes frame = {static_cast<uint8_t>(pending ? 0x79 : 0x69), 0xec, seq};
  frame.insert(frame.end(), dst.begin(), dst.end());
  frame.insert(frame.end(), src.begin(), src.begin() + tadogado::kPrefixLen);
  const Bytes start = {0x00, 0x01, 0x02, 0x03};
  frame.insert(frame.end(), start.begin(), start.end());
  appendLe(frame, counter, 4);
  frame.insert(frame.end(), datagram.begin(), datagram.end());
  appendLe(frame, counter & 0xFFFF, 2);
  appendLe(frame, static_cast<uint32_t>(seq) * 7 + counter, 4);
  return frame;
}

inline Bytes broadcastFrame(uint8_t seq, const Bytes& src, uint32_t counter, size_t size = 40) {
  Bytes frame = {0x49, 0xe8, seq, 0xff, 0xff};
  frame.insert(frame.end(), src.begin(), src.end());
  frame.push_back(0x05);
  appendLe(frame, counter & 0xFFFF, 2);
  frame.resize(frame.size() + size, 0);
  return frame;
}

inline Bytes enhAck(uint8_t seq, const Bytes& dst, const Bytes& src, uint16_t phase) {
  Bytes frame(tadogado::kEnhAckLen);
  tadogado::buildEnhAck(seq, dst.data(), src.data(), phase, frame.data());
  return frame;
}

inline Bytes wakeup(uint8_t seq, const Bytes& dstShort, uint16_t rendezvous) {
  Bytes frame(tadogado::kWakeupLen);
  tadogado::buildWakeup(seq, dstShort.data(), rendezvous, frame.data());
  return frame;
}

inline FrameClass classOf(const Bytes& frame) { return tadogado::classify(frame.data(), frame.size()); }

inline int64_t airtime(size_t length) { return tadogado::airtimeUs(length); }

class World;

struct Transmission {
  const class Node* sender;
  Bytes frame;
  int64_t start;
  int64_t end;
};

class Node {
 public:
  explicit Node(std::string name) : name(std::move(name)) {}
  virtual ~Node() = default;

  virtual bool listenedThrough(int64_t start, int64_t /*end*/) const { return txUntil <= start; }
  virtual void tick() {}
  virtual void receive(const Bytes& /*frame*/, int64_t /*now*/) {}

  // How strongly this node hears each sender, by name; -80 dBm unless set.
  float rssiOf(const Node& sender) const {
    const auto found = rssiFrom.find(sender.name);
    return found == rssiFrom.end() ? -80.0f : found->second;
  }

  std::string name;
  World* world = nullptr;
  std::map<std::string, float> rssiFrom;
  float incomingRssiDbm = -80.0f;  // Of the frame receive() is given.
  bool blocked = false;
  int64_t txUntil = 0;
  std::deque<Transmission> recentTx;
};

class World {
 public:
  template <typename T>
  T& add(T* node) {
    node->world = this;
    nodes_.emplace_back(node);
    return *node;
  }

  void link(std::initializer_list<const char*> names) {
    const std::vector<std::string> list(names.begin(), names.end());
    for (size_t i = 0; i < list.size(); i++) {
      for (size_t j = i + 1; j < list.size(); j++) {
        links_.insert(key(list[i], list[j]));
      }
    }
  }

  bool hears(const Node& a, const Node& b) const { return links_.count(key(a.name, b.name)) != 0; }

  int64_t transmit(Node& node, const Bytes& frame) {
    if (onTransmit) {
      onTransmit(node, frame);
    }
    const int64_t end = now + airtime(frame.size());
    onAir_.push_back({&node, frame, now, end});
    log.push_back({node.name, frame});
    node.txUntil = end;
    return end;
  }

  void run(int64_t durationUs) { runUntil(now + durationUs); }

  void runUntil(int64_t deadline) {
    while (now < deadline) {
      now += kTickUs;
      deliver();
      for (auto& node : nodes_) {
        if (!node->blocked) {
          node->tick();
        }
      }
    }
  }

  // Frames each sender put on air, in order.
  std::vector<Bytes> sentBy(const std::string& name) const {
    std::vector<Bytes> out;
    for (const auto& entry : log) {
      if (entry.first == name) {
        out.push_back(entry.second);
      }
    }
    return out;
  }

  int64_t now = 0;
  std::vector<std::pair<std::string, Bytes>> log;
  // (frame, node): corrupt that frame once at that node.
  std::vector<std::pair<Bytes, std::string>> lostFrames;
  std::function<void(Node&, const Bytes&)> onTransmit;

 private:
  static std::string key(const std::string& a, const std::string& b) { return a < b ? a + "|" + b : b + "|" + a; }

  // A frame survives an overlap with one at least kCaptureDb weaker, as a real radio does.
  static constexpr float kCaptureDb = 10.0f;
  bool overlaps(const Node& node, const Transmission& tx, const Transmission& other) const {
    return hears(node, *other.sender) && other.start < tx.end && other.end > tx.start &&
           node.rssiOf(*other.sender) > node.rssiOf(*tx.sender) - kCaptureDb;
  }

  void deliver() {
    std::vector<Transmission> ended;
    for (auto it = onAir_.begin(); it != onAir_.end();) {
      if (it->end <= now) {
        ended.push_back(*it);
        it = onAir_.erase(it);
      } else {
        ++it;
      }
    }
    for (const Transmission& tx : ended) {
      for (auto& node : nodes_) {
        if (node.get() == tx.sender || !hears(*node, *tx.sender) || !node->listenedThrough(tx.start, tx.end)) {
          continue;
        }
        bool collided = false;
        for (const Transmission& other : onAir_) {
          collided = collided || overlaps(*node, tx, other);
        }
        for (const Transmission& other : ended) {
          collided = collided || (&other != &tx && overlaps(*node, tx, other));
        }
        for (const Transmission& other : node->recentTx) {
          collided = collided || overlaps(*node, tx, other);
        }
        if (collided) {
          continue;
        }
        const auto lost = std::find(lostFrames.begin(), lostFrames.end(), std::make_pair(tx.frame, node->name));
        if (lost != lostFrames.end()) {
          lostFrames.erase(lost);
          continue;
        }
        node->incomingRssiDbm = node->rssiOf(*tx.sender);
        node->receive(tx.frame, now);
      }
      for (auto& node : nodes_) {
        node->recentTx.push_back(tx);
        while (node->recentTx.size() > 8) {
          node->recentTx.pop_front();
        }
      }
    }
  }

  std::vector<std::unique_ptr<Node>> nodes_;
  std::set<std::string> links_;
  std::deque<Transmission> onAir_;
};

struct UplinkResult {
  uint8_t seq;
  bool acked;
};

struct DownlinkResult {
  uint8_t seq;
  bool acked;
  int attempts;
};

// A battery device: asleep but for a sample each second, and a rendezvous.
class SimValve : public Node {
 public:
  SimValve(std::string name, Bytes eui, int64_t phaseUs) : Node(std::move(name)), eui(std::move(eui)), phase(phaseUs) {}

  bool listenedThrough(int64_t start, int64_t end) const override {
    if (txUntil > start) {
      return false;
    }
    if (hasPending_ && pending_.ackBy >= end && end > pending_.sent) {
      return true;
    }
    if (rendezvousAt_ <= end && end <= rendezvousUntil_) {
      return true;
    }
    return modPeriod(start - phase) + (end - start) < kSampleWindowUs;
  }

  void sendUplink(const Bytes& datagram) { outbox_.push_back(datagram); }

  void tick() override {
    const int64_t now = world->now;
    if (hasAckDue_ && now >= ackDueAt_) {
      world->transmit(*this, ackDue_);
      hasAckDue_ = false;
    }
    if (hasPending_ && now > pending_.ackBy) {
      if (pending_.attempts >= 3) {
        uplinkResults.push_back({pending_.seq, false});
        hasPending_ = false;
      } else if (now >= pending_.sent + kRetryDelayUs) {
        retries++;
        transmitPending();
      }
    }
    if (!hasPending_ && !outbox_.empty()) {
      pending_ = {nextSeq_, outbox_.front(), 0, 0, 0};
      outbox_.pop_front();
      hasPending_ = true;
      nextSeq_++;
      transmitPending();
    }
  }

  void receive(const Bytes& frame, int64_t now) override {
    const uint8_t* bytes = frame.data();
    switch (classOf(frame)) {
      case FrameClass::Wakeup: {
        tadogado::WakeUp wake;
        if (!tadogado::parseWakeup(bytes, frame.size(), wake)) {
          return;
        }
        const Bytes dst(wake.dstShort, wake.dstShort + 2);
        if (dst == shortAddress(eui) || dst == Bytes{0xff, 0xff}) {
          rendezvousAt_ = now + wake.rendezvous * kUnitUs - 2000;
          rendezvousUntil_ = now + wake.rendezvous * kUnitUs + kRendezvousWindowUs;
        }
        return;
      }
      case FrameClass::EnhAck: {
        tadogado::EnhAck ack;
        if (tadogado::parseEnhAck(bytes, frame.size(), ack) && Bytes(ack.dstEui, ack.dstEui + 8) == eui &&
            hasPending_ && ack.seq == pending_.seq) {
          ackDelaysUs.push_back(now - airtime(frame.size()) - pending_.sent);
          uplinkResults.push_back({ack.seq, true});
          hasPending_ = false;
        }
        return;
      }
      case FrameClass::Data:
      case FrameClass::DataPending: {
        tadogado::DataFrame data;
        if (!tadogado::parseData(bytes, frame.size(), data) || Bytes(data.dstEui, data.dstEui + 8) != eui) {
          return;
        }
        if (receivedSeqs_.insert(data.seq).second) {
          received.push_back(frame);
        }
        Bytes src(data.srcPrefix, data.srcPrefix + tadogado::kPrefixLen);
        src.insert(src.end(), kEuiTail.begin(), kEuiTail.end());
        const auto phaseUnits = static_cast<uint16_t>((modPeriod(now - phase) / kUnitUs) % tadogado::kCslPeriodUnits);
        ackDue_ = enhAck(data.seq, src, eui, phaseUnits);
        ackDueAt_ = now + kAckDelayUs;
        hasAckDue_ = true;
        return;
      }
      case FrameClass::Broadcast:
        advertisements++;
        return;
      default:
        return;
    }
  }

  Bytes eui;
  int64_t phase;
  std::vector<Bytes> received;
  std::vector<UplinkResult> uplinkResults;
  std::vector<int64_t> ackDelaysUs;
  int retries = 0;
  int advertisements = 0;

 private:
  struct Pending {
    uint8_t seq;
    Bytes datagram;
    int attempts;
    int64_t sent;
    int64_t ackBy;
  };

  void transmitPending() {
    pending_.attempts++;
    counter_++;
    const int64_t end = world->transmit(*this, dataFrame(pending_.seq, kBridgeEui, eui, counter_, pending_.datagram));
    pending_.sent = end;
    pending_.ackBy = end + kValveAckWaitUs;
  }

  uint32_t counter_ = 1000;
  uint8_t nextSeq_ = 10;
  std::deque<Bytes> outbox_;
  bool hasPending_ = false;
  Pending pending_ = {};
  int64_t rendezvousAt_ = -1;
  int64_t rendezvousUntil_ = 0;
  bool hasAckDue_ = false;
  int64_t ackDueAt_ = 0;
  Bytes ackDue_;
  std::set<uint8_t> receivedSeqs_;
};

// The coordinator: its receiver is always on, it sends trains to sleeping valves.
class SimBridge : public Node {
 public:
  SimBridge() : Node("bridge") {}

  void sendDownlink(const Bytes& valveEui, const Bytes& datagram, int64_t sampleOffsetUs) {
    Job job;
    job.eui = valveEui;
    job.datagram = datagram;
    job.seq = ++seq_;
    job.offset = sampleOffsetUs;
    jobs_.push_back(job);
  }

  void sendAdvertisement() {
    Job job;
    job.broadcast = true;
    job.seq = ++seq_;
    jobs_.push_back(job);
  }

  void tick() override {
    const int64_t now = world->now;
    if (hasAckDue_ && now >= ackDueAt_) {
      world->transmit(*this, ackDue_);
      hasAckDue_ = false;
    }
    if (!active_) {
      if (!jobs_.empty() && jobs_.front().notBefore <= now) {
        job_ = jobs_.front();
        jobs_.pop_front();
        start(now);
      }
      return;
    }
    Job& job = job_;
    if (job.state == State::Train) {
      if (now >= job.end) {
        world->transmit(*this, wakeup(job.seq, job.dstShort, 0));
        job.state = State::Data;
      } else if (now >= job.nextWake && now >= job.start) {
        const auto rendezvous = static_cast<uint16_t>((job.end - now) / kUnitUs);
        world->transmit(*this, wakeup(job.seq, job.dstShort, rendezvous));
        job.nextWake += kWakeStepUs;
      }
    } else if (job.state == State::Data && now >= txUntil) {
      job.ackBy = world->transmit(*this, job.frame) + 30000;
      job.state = State::Ack;
    } else if (job.state == State::Ack) {
      if (job.broadcast) {
        if (now >= txUntil) {
          active_ = false;
        }
        return;
      }
      if (now > job.ackBy) {
        active_ = false;
        if (job.attempts >= 3) {
          downlinkResults.push_back({job.seq, false, job.attempts});
        } else {
          job.notBefore = now + kRetryDelayUs;
          jobs_.push_front(job);
        }
      }
    }
  }

  void receive(const Bytes& frame, int64_t now) override {
    const FrameClass frameClass = classOf(frame);
    if (frameClass == FrameClass::Data || frameClass == FrameClass::DataPending) {
      tadogado::DataFrame data;
      if (!tadogado::parseData(frame.data(), frame.size(), data) || Bytes(data.dstEui, data.dstEui + 8) != kBridgeEui) {
        return;
      }
      Bytes src(data.srcPrefix, data.srcPrefix + tadogado::kPrefixLen);
      if (uplinkKeys_.insert({src, data.seq}).second) {
        uplinks.push_back(frame);
      }
      src.insert(src.end(), kEuiTail.begin(), kEuiTail.end());
      ackDue_ = enhAck(data.seq, src, kBridgeEui, 40000);
      ackDueAt_ = now + kAckDelayUs;
      hasAckDue_ = true;
    } else if (frameClass == FrameClass::EnhAck) {
      tadogado::EnhAck ack;
      if (active_ && job_.state == State::Ack && tadogado::parseEnhAck(frame.data(), frame.size(), ack) &&
          Bytes(ack.dstEui, ack.dstEui + 8) == kBridgeEui && ack.seq == job_.seq) {
        downlinkResults.push_back({ack.seq, true, job_.attempts});
        active_ = false;
      }
    }
  }

  std::vector<Bytes> uplinks;
  std::vector<DownlinkResult> downlinkResults;

 private:
  enum class State { Train, Data, Ack };

  struct Job {
    bool broadcast = false;
    Bytes eui;
    Bytes datagram;
    uint8_t seq = 0;
    int64_t offset = 0;
    int attempts = 0;
    int64_t notBefore = 0;
    int64_t start = 0;
    int64_t end = 0;
    int64_t nextWake = 0;
    int64_t ackBy = 0;
    Bytes dstShort;
    Bytes frame;
    State state = State::Train;
  };

  void start(int64_t now) {
    Job& job = job_;
    job.attempts++;
    counter_++;
    if (job.broadcast) {
      job.start = now;
      job.end = now + 1000000;
      job.dstShort = {0xff, 0xff};
      job.frame = broadcastFrame(job.seq, kBridgeEui, counter_);
    } else {
      // The first sample of the valve at least 50 ms from now.
      const int64_t periods = floorDiv(now + 50000 - job.offset, kPeriodUs) + 1;
      const int64_t sample = job.offset + periods * kPeriodUs;
      job.start = sample - 40000;
      job.end = sample + 40000;
      job.dstShort = shortAddress(job.eui);
      job.frame = dataFrame(job.seq, job.eui, kBridgeEui, counter_, job.datagram);
    }
    job.state = State::Train;
    job.nextWake = job.start;
    active_ = true;
  }

  uint8_t seq_ = 100;
  uint32_t counter_ = 5000;
  std::deque<Job> jobs_;
  bool active_ = false;
  Job job_;
  bool hasAckDue_ = false;
  int64_t ackDueAt_ = 0;
  Bytes ackDue_;
  std::set<std::pair<Bytes, uint8_t>> uplinkKeys_;
};

// Two ends of a network link with a fixed delay.
class LoopbackPeer : public tadogado::PeerSink {
 public:
  LoopbackPeer(World& world, int64_t delayUs) : world_(world), delayUs_(delayUs) {}

  void sendFrame(bool uplink, const uint8_t* frame, size_t length) override {
    other->queue_.push_back({world_.now + delayUs_, uplink, Bytes(frame, frame + length)});
  }

  template <typename F>
  void poll(F&& deliver) {
    while (!queue_.empty() && queue_.front().at <= world_.now) {
      const Message message = queue_.front();
      queue_.pop_front();
      deliver(message.uplink, message.frame);
    }
  }

  LoopbackPeer* other = nullptr;

 private:
  struct Message {
    int64_t at;
    bool uplink;
    Bytes frame;
  };

  World& world_;
  int64_t delayUs_;
  std::deque<Message> queue_;
};

// A proxy node: the real engine on a simulated half-duplex radio.
class SimProxy : public Node, public tadogado::RadioPort {
 public:
  SimProxy(std::string name, const Bytes* bridge) : Node(std::move(name)), engine(*this, monitor) {
    if (bridge != nullptr) {
      monitor.setBridge(bridge->data());
    }
  }

  void serve(const Bytes& prefix) { monitor.setProxied(prefix.data(), true); }

  // The Hellos of a real pair carry each node's capacity every 2 s; here it is current.
  void pairWith(LoopbackPeer& link, tadogado::Role role, SimProxy& partner) {
    partner_ = &partner;
    peer_ = &link;
    engine.setRole(role);
    engine.setPeer(&link);
  }

  // RadioPort
  bool send(const uint8_t* frame, size_t length) override {
    sent++;
    sleepUntil(static_cast<uint64_t>(world->transmit(*this, Bytes(frame, frame + length))));
    return true;
  }

  void sleepUntil(uint64_t timeUs) override {
    blocked = true;
    world->runUntil(static_cast<int64_t>(timeUs));
    blocked = false;
  }

  uint64_t nowUs() override { return static_cast<uint64_t>(world->now); }

  // Node
  void receive(const Bytes& frame, int64_t now) override { inbox_.push_back({frame, now, incomingRssiDbm}); }

  void tick() override {
    if (partner_ != nullptr) {
      engine.setPeerCapacity(partner_->engine.capacity(nowUs()), nowUs());
    }
    if (!inbox_.empty()) {
      const auto received = inbox_.front();
      inbox_.pop_front();
      tadogado::RxFrame rx = {};
      std::memcpy(rx.payload, received.frame.data(), received.frame.size());
      rx.length = received.frame.size();
      rx.rssiDbm = received.rssiDbm;
      rx.lqi = 40;
      rx.crcOk = true;
      rx.timestampUs = static_cast<uint64_t>(received.now);
      engine.onFrame(rx);
    }
    if (peer_ != nullptr) {
      peer_->poll([this](bool uplink, const Bytes& frame) {
        engine.onPeerFrame(uplink, frame.data(), frame.size(), static_cast<uint64_t>(world->now));
      });
    }
    engine.tick();
  }

  tadogado::LinkMonitor monitor;
  tadogado::ProxyEngine engine;
  int sent = 0;

 private:
  struct Incoming {
    Bytes frame;
    int64_t now;
    float rssiDbm;
  };
  std::deque<Incoming> inbox_;
  LoopbackPeer* peer_ = nullptr;
  SimProxy* partner_ = nullptr;
};

}  // namespace sim
