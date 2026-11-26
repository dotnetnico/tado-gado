// Two nodes that find each other and share the work.
//
// Nodes with the same admin token find each other with a broadcast on the home network.
// The node that hears the Tado bridge best becomes the bridge node, the other the valve
// node. A node that does not hear the bridge at all can only be the valve node, which is
// the usual case: it stands near valves the bridge cannot reach.
//
// The roles follow from the measurements, but transmitting takes a decision of the user.
// Both nodes start in monitor. Switching one to proxy switches the other too, and a
// restart of either stops both, so pulling a plug remains an off switch. If the link
// between them drops, both stop answering until it is back: an ACK promises delivery,
// and alone a node cannot deliver.
//
// The nodes also share what they measure and what the user set: the ACKs each heard,
// so both count a frame as answered when either heard the answer, and the names of the
// devices.
#pragma once

#include <cstddef>
#include <cstdint>

#include "link_monitor.h"
#include "name_table.h"
#include "pair_protocol.h"
#include "proxy_engine.h"

namespace tadogado {

class PairTransport {
 public:
  virtual ~PairTransport() = default;
  virtual void broadcast(const uint8_t* data, size_t length) = 0;
  virtual void sendToPeer(const uint8_t* data, size_t length) = 0;
  // The sender of the datagram being handled is the peer.
  virtual void rememberPeer() = 0;
};

// What the dashboard shows of a pair: a copy, so it can be formatted without a lock.
struct PairStatus {
  bool enabled;
  bool hasPeer;
  uint8_t peerId[kNodeIdLen];
  bool peerAlive;
  bool rolesDecided;
  uint32_t peerRestarts;
  uint32_t rejected;
  bool hasOwnRssi;
  float ownRssi;
  bool hasPeerRssi;
  float peerRssi;
  bool wanted;
  Role role;
};

class PairManager : public PeerSink {
 public:
  PairManager(PairTransport& transport, ProxyEngine& engine, LinkMonitor& monitor)
      : transport_(transport), engine_(engine), monitor_(monitor) {}

  // Without a secret the node stays on its own, and can still proxy as a single node.
  void begin(const uint8_t* secret, size_t secretLength, const uint8_t* nodeId, uint32_t bootNumber);
  bool enabled() const { return secretLength_ > 0; }

  void tick(uint64_t nowUs);
  void onDatagram(const uint8_t* data, size_t length, uint64_t nowUs);

  // Switch proxy on or off. Returns false if the node may not send, or if a peer is
  // there but the roles are not decided, because neither node hears the bridge yet. The
  // engine follows on the next tick, so this is safe from another task while a
  // transmission is under way.
  bool requestProxy(bool enabled, uint64_t nowUs);
  // False while the radio is tuned outside the band the proxy may send in (see band.h).
  // The node then refuses to start, and stops both nodes if it was proxying.
  void setTransmitAllowed(bool allowed, uint64_t nowUs);
  bool transmitAllowed() const { return transmitAllowed_; }
  // Send the list of proxied devices to the peer.
  void syncProxied();
  // True once after the peer replaced the list, so it can be saved.
  bool takeProxiedChanged();

  // The names to share. Each node sends its names when it finds the other; a node keeps
  // its own name for a device. A name the user sets or clears goes to the other node and
  // replaces its name there.
  void setNames(NameTable* names) { names_ = names; }
  void sendName(const uint8_t* prefix, const char* name);
  // True once after the peer changed a name, so the names can be saved.
  bool takeNamesChanged();

  void sendFrame(bool uplink, const uint8_t* frame, size_t length) override;

  Role role() const { return role_; }
  bool wanted() const { return wanted_; }
  // True while frames are really proxied.
  bool active() const { return engine_.proxyMode(); }
  bool hasPeer() const { return hasPeer_; }
  bool peerAlive(uint64_t nowUs) const;
  bool rolesDecided() const { return hasAssignment_; }
  const uint8_t* peerId() const { return peerId_; }
  uint64_t peerAgeUs(uint64_t nowUs) const { return nowUs - peerSeenUs_; }
  bool peerRssi(float& out) const;
  bool ownRssi(uint64_t nowUs, float& out) const;
  uint32_t peerRestarts() const { return peerRestarts_; }
  uint32_t rejected() const { return rejected_; }
  PairStatus status(uint64_t nowUs) const;

 private:
  void sendHello(uint64_t nowUs);
  void send(PairType type, const uint8_t* payload, size_t length, bool broadcast);
  void sendMode(bool enabled);
  void onHello(const PairHeader& header, const uint8_t* payload, size_t length, uint64_t nowUs);
  void onPeerMessage(const PairHeader& header, const uint8_t* payload, size_t length, uint64_t nowUs);
  void onProxied(const uint8_t* payload, size_t length);
  void onAcks(const uint8_t* payload, size_t length, uint64_t nowUs);
  void onName(const uint8_t* payload, size_t length);
  void sendAcks(uint64_t nowUs);
  void sendNameEntry(const uint8_t* prefix, const char* name, bool replace);
  void syncNames();
  void elect(uint64_t nowUs);
  void updateRole(uint64_t nowUs);
  void apply(uint64_t nowUs);
  bool isDecider() const;

  PairTransport& transport_;
  ProxyEngine& engine_;
  LinkMonitor& monitor_;

  uint8_t secret_[kPairMaxSecret] = {};
  size_t secretLength_ = 0;
  uint8_t nodeId_[kNodeIdLen] = {};
  uint32_t boot_ = 0;
  uint32_t counter_ = 0;

  bool hasPeer_ = false;
  uint8_t peerId_[kNodeIdLen] = {};
  uint32_t peerBoot_ = 0;
  // Separate, as an access point can delay a broadcast behind a later unicast.
  uint32_t peerCounter_ = 0;
  uint32_t peerHelloCounter_ = 0;
  uint64_t peerSeenUs_ = 0;
  bool peerHasRssi_ = false;
  float peerRssi_ = 0;
  bool peerWanted_ = false;

  bool hasAssignment_ = false;
  uint8_t bridgeNode_[kNodeIdLen] = {};
  Role role_ = Role::Single;

  bool transmitAllowed_ = true;
  bool wanted_ = false;
  bool startPending_ = false;
  uint64_t startDeadlineUs_ = 0;
  uint64_t nextHelloUs_ = 0;
  bool proxiedChanged_ = false;
  NameTable* names_ = nullptr;
  size_t nameSyncNext_ = NameTable::kCapacity;  // Nothing to send until a peer appears.
  size_t nameRepeatNext_ = 0;
  bool namesChanged_ = false;
  uint64_t nextAcksUs_ = 0;
  uint32_t peerRestarts_ = 0;
  uint32_t rejected_ = 0;
};

}  // namespace tadogado
