// End-to-end runs of the proxy engine in the simulated network of sim.h.
#include <unity.h>

#include <cstring>
#include <string>

#include "name_table.h"
#include "sim.h"
#include "status_json.h"

using namespace sim;

namespace {

const Bytes kBridge = prefixOf(kBridgeEui);
const Bytes kFar = prefixOf(kFarEui);
const Bytes kNear = prefixOf(kNearEui);

struct Network {
  World world;
  SimBridge* hub = nullptr;
  SimValve* far = nullptr;
  SimValve* near = nullptr;
  SimProxy* proxy = nullptr;
};

// The bridge and the near valve hear each other. The far valve hears only the proxy.
Network* build(bool proxyMode, std::initializer_list<Bytes> proxied = {kFar}, bool knowsBridge = true) {
  auto* net = new Network;
  net->hub = &net->world.add(new SimBridge());
  net->far = &net->world.add(new SimValve("far", kFarEui, seconds(0.3)));
  net->near = &net->world.add(new SimValve("near", kNearEui, seconds(0.7)));
  net->proxy = &net->world.add(new SimProxy("relay", knowsBridge ? &kBridge : nullptr));
  for (const Bytes& device : proxied) {
    net->proxy->serve(device);
  }
  net->proxy->engine.setProxyMode(proxyMode);
  net->world.link({"bridge", "near", "relay"});
  net->world.link({"far", "relay"});
  return net;
}

size_t countClass(const std::vector<Bytes>& frames, FrameClass frameClass) {
  size_t count = 0;
  for (const Bytes& frame : frames) {
    count += classOf(frame) == frameClass ? 1 : 0;
  }
  return count;
}

const tadogado::DeviceLink& linkOf(const SimProxy& proxy, const Bytes& prefix) {
  const tadogado::DeviceLink* link = proxy.monitor.findLink(prefix.data());
  TEST_ASSERT_NOT_NULL(link);
  return *link;
}

void assertUplinks(const SimValve& valve, std::initializer_list<UplinkResult> expected) {
  TEST_ASSERT_EQUAL(expected.size(), valve.uplinkResults.size());
  size_t i = 0;
  for (const UplinkResult& result : expected) {
    TEST_ASSERT_EQUAL_UINT8(result.seq, valve.uplinkResults[i].seq);
    TEST_ASSERT_EQUAL(result.acked, valve.uplinkResults[i].acked);
    i++;
  }
}

std::string statusOf(SimProxy& proxy, const tadogado::NameTable* names = nullptr) {
  static char buffer[32000];
  tadogado::StatusInput input = {};
  input.monitor = &proxy.monitor;
  input.engine = &proxy.engine;
  input.names = names;
  input.firmware = "sim";
  input.variant = "sim";
  input.nowUs = proxy.nowUs();
  TEST_ASSERT_TRUE(tadogado::writeStatusJson(input, buffer, sizeof(buffer)));
  return buffer;
}

void assertContains(const std::string& text, const char* part) {
  TEST_ASSERT_TRUE_MESSAGE(text.find(part) != std::string::npos, part);
}

void test_without_a_proxy_the_far_valve_gets_nothing_through() {
  Network* net = build(false);
  net->far->sendUplink(bytesOf("report"));
  net->hub->sendDownlink(kFarEui, bytesOf("set point"), seconds(0.3));
  net->world.run(seconds(9.0));
  TEST_ASSERT_EQUAL(0, net->hub->uplinks.size());
  assertUplinks(*net->far, {{10, false}});
  TEST_ASSERT_EQUAL(0, net->far->received.size());
  TEST_ASSERT_FALSE(net->hub->downlinkResults.back().acked);
  delete net;
}

void test_monitor_mode_never_transmits() {
  Network* net = build(false, {kFar, kNear});
  net->far->sendUplink(bytesOf("report"));
  net->world.run(seconds(0.5));
  net->near->sendUplink(bytesOf("report"));
  net->hub->sendDownlink(kNearEui, bytesOf("set point"), seconds(0.7));
  net->hub->sendAdvertisement();
  net->world.run(seconds(9.0));
  TEST_ASSERT_EQUAL(0, net->proxy->sent);
  TEST_ASSERT_EQUAL(0, net->proxy->engine.counters().framesTx);
  delete net;
}

void test_monitor_mode_measures_the_links() {
  Network* net = build(false);
  net->near->sendUplink(bytesOf("report"));
  net->world.run(seconds(0.5));
  net->far->sendUplink(bytesOf("report"));
  net->world.run(seconds(9.0));
  const tadogado::DeviceLink& near = linkOf(*net->proxy, kNear);
  TEST_ASSERT_EQUAL(1, near.uplinksAcked);
  TEST_ASSERT_EQUAL(0, near.uplinksUnacked);
  const tadogado::DeviceLink& far = linkOf(*net->proxy, kFar);
  TEST_ASSERT_EQUAL(0, far.uplinksAcked);
  TEST_ASSERT_GREATER_THAN(0, far.uplinksUnacked);
  TEST_ASSERT_EQUAL(2, far.retransmissions);
  delete net;
}

void test_uplink_of_a_far_valve_reaches_the_bridge_in_one_try() {
  Network* net = build(true);
  net->far->sendUplink(bytesOf("report"));
  net->world.run(seconds(1.0));
  TEST_ASSERT_EQUAL(1, net->hub->uplinks.size());
  assertUplinks(*net->far, {{10, true}});
  TEST_ASSERT_EQUAL(0, net->far->retries);
  TEST_ASSERT_EQUAL(1, net->proxy->engine.counters().uplinksForwarded);
  delete net;
}

void test_proxied_ack_lands_in_the_documented_window() {
  Network* net = build(true);
  net->far->sendUplink(bytesOf("report"));
  net->world.run(seconds(1.0));
  TEST_ASSERT_EQUAL(1, net->far->ackDelaysUs.size());
  TEST_ASSERT_INT64_WITHIN(500, 5000, net->far->ackDelaysUs[0]);
  delete net;
}

void test_downlink_reaches_a_sleeping_valve_byte_for_byte() {
  Network* net = build(true);
  net->hub->sendDownlink(kFarEui, bytesOf("set point"), seconds(0.3));
  net->world.run(seconds(4.0));
  TEST_ASSERT_EQUAL(1, net->hub->downlinkResults.size());
  TEST_ASSERT_EQUAL_UINT8(101, net->hub->downlinkResults[0].seq);
  TEST_ASSERT_TRUE(net->hub->downlinkResults[0].acked);
  TEST_ASSERT_EQUAL(1, net->hub->downlinkResults[0].attempts);
  std::vector<Bytes> fromBridge;
  for (const Bytes& frame : net->world.sentBy("bridge")) {
    if (classOf(frame) == FrameClass::Data) {
      fromBridge.push_back(frame);
    }
  }
  TEST_ASSERT_TRUE(net->far->received == fromBridge);
  TEST_ASSERT_EQUAL(1, net->proxy->engine.counters().downlinksDelivered);
  delete net;
}

void test_second_downlink_uses_a_short_train() {
  Network* net = build(true);
  net->hub->sendDownlink(kFarEui, bytesOf("first"), seconds(0.3));
  net->world.run(seconds(4.0));
  const size_t fullTrain = countClass(net->world.sentBy("relay"), FrameClass::Wakeup);
  TEST_ASSERT_TRUE(net->proxy->engine.anchors().contains(kFar.data()));
  net->hub->sendDownlink(kFarEui, bytesOf("second"), seconds(0.3));
  net->world.run(seconds(4.0));
  const size_t allWakes = countClass(net->world.sentBy("relay"), FrameClass::Wakeup);
  TEST_ASSERT_EQUAL(2, net->far->received.size());
  TEST_ASSERT_GREATER_THAN(200, fullTrain);
  TEST_ASSERT_LESS_THAN(30, allWakes - fullTrain);
  delete net;
}

void test_retry_after_a_lost_ack_is_answered_but_not_delivered_twice() {
  Network* net = build(true);
  bool lost = false;
  World& world = net->world;
  world.onTransmit = [&](Node& node, const Bytes& frame) {
    if (node.name == "relay" && classOf(frame) == FrameClass::EnhAck && !lost) {
      lost = true;
      world.lostFrames.push_back({frame, "far"});
    }
  };
  net->far->sendUplink(bytesOf("report"));
  world.run(seconds(4.0));
  TEST_ASSERT_EQUAL(1, net->far->retries);
  assertUplinks(*net->far, {{10, true}});
  TEST_ASSERT_EQUAL(1, net->hub->uplinks.size());
  TEST_ASSERT_EQUAL(1, countClass(world.sentBy("relay"), FrameClass::Data));
  delete net;
}

void test_devices_that_were_not_named_are_left_alone() {
  Network* net = build(true, {kFar});
  net->near->sendUplink(bytesOf("report"));
  net->hub->sendDownlink(kNearEui, bytesOf("set point"), seconds(0.7));
  net->world.run(seconds(4.0));
  TEST_ASSERT_EQUAL(0, net->proxy->sent);
  TEST_ASSERT_TRUE(net->hub->downlinkResults[0].acked);
  delete net;
}

void test_no_proxied_ack_with_an_empty_budget() {
  Network* net = build(true);
  net->proxy->engine.duty().record(tadogado::DutyCycleLimiter::kBudgetUs, 1);
  net->far->sendUplink(bytesOf("report"));
  net->world.run(seconds(9.0));
  TEST_ASSERT_EQUAL(0, net->proxy->sent);
  TEST_ASSERT_EQUAL(3, net->proxy->engine.counters().acksWithheld);
  assertUplinks(*net->far, {{10, false}});
  delete net;
}

void test_refused_frame_is_taken_when_budget_returns_for_its_retry() {
  // A retry of a refused frame must not be ACKed and then forgotten. The budget was
  // spent in the first minute and frees at 61 minutes, between attempt one and two.
  Network* net = build(true);
  net->proxy->engine.duty().record(tadogado::DutyCycleLimiter::kBudgetUs, 1);
  net->world.now = seconds(61 * 60.0) - seconds(0.5);
  net->far->sendUplink(bytesOf("report"));
  net->world.run(seconds(1.0));
  TEST_ASSERT_EQUAL(1, net->proxy->engine.counters().acksWithheld);
  net->world.run(seconds(4.0));
  assertUplinks(*net->far, {{10, true}});
  TEST_ASSERT_EQUAL(1, net->hub->uplinks.size());
  delete net;
}

void test_transmissions_stay_within_the_budget() {
  Network* net = build(true);
  const int64_t leftUs = seconds(2.0);
  net->proxy->engine.duty().record(tadogado::DutyCycleLimiter::kBudgetUs - leftUs, 1);
  for (uint8_t index = 0; index < 8; index++) {
    net->hub->sendDownlink(kFarEui, Bytes(8, index), seconds(0.3));
  }
  net->world.run(seconds(30.0));
  int64_t air = 0;
  for (const Bytes& frame : net->world.sentBy("relay")) {
    air += airtime(frame.size());
  }
  TEST_ASSERT_GREATER_THAN(0, air);
  TEST_ASSERT_LESS_OR_EQUAL(leftUs, air);
  // Every ACK was a promise the budget could keep.
  TEST_ASSERT_EQUAL(0, net->proxy->engine.counters().dutyBlocked);
  TEST_ASSERT_EQUAL(0, net->proxy->engine.counters().deliveriesFailed);
  delete net;
}

void test_router_advertisement_reaches_the_far_valve_at_a_limited_rate() {
  Network* net = build(true);
  net->hub->sendAdvertisement();
  net->world.run(seconds(3.5));
  TEST_ASSERT_EQUAL(1, net->far->advertisements);
  net->hub->sendAdvertisement();
  net->world.run(seconds(3.5));
  TEST_ASSERT_EQUAL(1, net->proxy->engine.counters().advertisementsForwarded);
  delete net;
}

void test_bridge_is_learned_from_two_trains() {
  Network* net = build(false, {kFar}, false);
  net->hub->sendDownlink(kNearEui, bytesOf("set point"), seconds(0.7));
  net->world.run(seconds(2.5));
  TEST_ASSERT_FALSE(net->proxy->monitor.hasBridge());  // One train could be forged.
  net->hub->sendDownlink(kNearEui, bytesOf("set point 2"), seconds(0.7));
  net->world.run(seconds(2.5));
  TEST_ASSERT_TRUE(net->proxy->monitor.hasBridge());
  TEST_ASSERT_EQUAL_UINT8_ARRAY(kBridge.data(), net->proxy->monitor.bridge(), tadogado::kPrefixLen);
  delete net;
}

void test_a_valve_can_be_found_by_its_downlink() {
  // The dashboard search: a command from the bridge points out the valve.
  Network* net = build(false);
  tadogado::NameTable names;
  TEST_ASSERT_TRUE(names.set(kNear.data(), "Bedroom"));
  net->hub->sendDownlink(kNearEui, bytesOf("hello"), seconds(0.7));
  net->world.run(seconds(2.0));
  const tadogado::DeviceLink& near = linkOf(*net->proxy, kNear);
  TEST_ASSERT_NOT_EQUAL(0, near.downlinks);
  TEST_ASSERT_LESS_THAN(seconds(2.0), net->proxy->nowUs() - near.lastDownlinkUs);
  assertContains(statusOf(*net->proxy, &names), "\"name\":\"Bedroom\"");
  delete net;
}

void test_status_holds_real_proxy_numbers() {
  Network* net = build(true);
  net->far->sendUplink(bytesOf("report"));
  net->world.run(seconds(1.0));
  tadogado::AckLatency latency;
  TEST_ASSERT_TRUE(net->proxy->engine.ackLatency(latency));
  TEST_ASSERT_EQUAL(1, latency.samples);
  TEST_ASSERT_GREATER_THAN(0, net->proxy->engine.duty().usedUs(net->proxy->nowUs()));
  TEST_ASSERT_EQUAL(1, linkOf(*net->proxy, kFar).proxiedUplinks);
  const std::string status = statusOf(*net->proxy);
  assertContains(status, "\"mode\":\"proxy\"");
  assertContains(status, "\"proxy_available\":true");
  assertContains(status, "\"proxied\":true");
  assertContains(status, "\"proxy_direction\":\"both\"");
  assertContains(status, "\"hour\":[");
  delete net;
}

struct PairNetwork {
  World world;
  SimBridge* hub = nullptr;
  SimValve* far = nullptr;
  SimProxy* bridgeNode = nullptr;
  SimProxy* valveNode = nullptr;
  LoopbackPeer* bridgeLink = nullptr;
  LoopbackPeer* valveLink = nullptr;

  ~PairNetwork() {
    delete bridgeLink;
    delete valveLink;
  }
};

// A node near the bridge and a node near the far valve, which hear only their own side.
PairNetwork* buildPair(int64_t peerDelayUs = 2000) {
  auto* net = new PairNetwork;
  net->hub = &net->world.add(new SimBridge());
  net->far = &net->world.add(new SimValve("far", kFarEui, seconds(0.3)));
  net->bridgeLink = new LoopbackPeer(net->world, peerDelayUs);
  net->valveLink = new LoopbackPeer(net->world, peerDelayUs);
  net->bridgeLink->other = net->valveLink;
  net->valveLink->other = net->bridgeLink;
  net->bridgeNode = &net->world.add(new SimProxy("bridge-node", &kBridge));
  net->valveNode = &net->world.add(new SimProxy("valve-node", &kBridge));
  net->bridgeNode->pairWith(*net->bridgeLink, tadogado::Role::Bridge, *net->valveNode);
  net->valveNode->pairWith(*net->valveLink, tadogado::Role::Valve, *net->bridgeNode);
  for (SimProxy* node : {net->bridgeNode, net->valveNode}) {
    node->serve(kFar);
    node->engine.setProxyMode(true);
  }
  net->world.link({"bridge", "bridge-node"});
  net->world.link({"far", "valve-node"});
  return net;
}

void test_pair_serves_both_directions() {
  PairNetwork* net = buildPair();
  net->far->sendUplink(bytesOf("report"));
  net->hub->sendDownlink(kFarEui, bytesOf("set point"), seconds(0.3));
  net->world.run(seconds(4.0));
  TEST_ASSERT_EQUAL(1, net->hub->uplinks.size());
  assertUplinks(*net->far, {{10, true}});
  TEST_ASSERT_EQUAL(1, net->far->received.size());
  TEST_ASSERT_EQUAL(1, net->hub->downlinkResults.size());
  TEST_ASSERT_TRUE(net->hub->downlinkResults[0].acked);
  TEST_ASSERT_EQUAL(1, net->hub->downlinkResults[0].attempts);
  delete net;
}

void test_pair_stays_available_while_the_other_node_sends_a_train() {
  PairNetwork* net = buildPair();
  net->hub->sendDownlink(kFarEui, bytesOf("first"), seconds(0.3));
  net->hub->sendDownlink(kFarEui, bytesOf("second"), seconds(0.3));
  net->world.run(seconds(9.0));
  TEST_ASSERT_EQUAL(2, net->far->received.size());
  TEST_ASSERT_EQUAL(1, net->hub->downlinkResults[1].attempts);
  delete net;
}

void test_pair_nodes_ignore_each_others_transmissions() {
  PairNetwork* net = buildPair();
  net->world.link({"bridge-node", "valve-node"});
  net->far->sendUplink(bytesOf("report"));
  net->hub->sendDownlink(kFarEui, bytesOf("set point"), seconds(0.3));
  net->world.run(seconds(6.0));
  TEST_ASSERT_EQUAL(1, net->hub->uplinks.size());
  TEST_ASSERT_EQUAL(1, net->far->received.size());
  const size_t data = countClass(net->world.sentBy("bridge-node"), FrameClass::Data) +
                      countClass(net->world.sentBy("valve-node"), FrameClass::Data);
  TEST_ASSERT_EQUAL(2, data);
  delete net;
}

// A valve that never answers: the valve node keeps trying for longer than an echo is
// remembered, and the bridge node hears every try.
void test_pair_bridge_node_does_not_answer_the_valve_node() {
  PairNetwork* net = buildPair();
  net->world.link({"bridge-node", "valve-node"});
  net->far->blocked = true;
  net->hub->sendDownlink(kFarEui, bytesOf("set point"), seconds(0.3));
  net->world.run(seconds(20.0));
  TEST_ASSERT_EQUAL(1, net->bridgeNode->engine.counters().acksSent);
  TEST_ASSERT_EQUAL(1, linkOf(*net->bridgeNode, kFar).proxiedDownlinks);
  TEST_ASSERT_EQUAL(3, net->valveNode->engine.counters().trains);
  TEST_ASSERT_EQUAL(0, net->valveNode->engine.counters().downlinksDelivered);
  TEST_ASSERT_EQUAL(1, net->valveNode->engine.counters().deliveriesFailed);
  TEST_ASSERT_FALSE(net->valveNode->engine.anchors().contains(kFar.data()));
  delete net;
}

void test_pair_valve_node_does_not_answer_the_bridge_node() {
  PairNetwork* net = buildPair();
  net->world.link({"bridge-node", "valve-node"});
  net->hub->blocked = true;
  net->far->sendUplink(bytesOf("report"));
  net->world.run(seconds(20.0));
  TEST_ASSERT_EQUAL(1, net->valveNode->engine.counters().acksSent);
  TEST_ASSERT_EQUAL(1, linkOf(*net->valveNode, kFar).proxiedUplinks);
  TEST_ASSERT_EQUAL(0, net->bridgeNode->engine.counters().uplinksForwarded);
  TEST_ASSERT_EQUAL(1, net->bridgeNode->engine.counters().deliveriesFailed);
  delete net;
}

// What the other node sends says nothing of the link to a valve or to the bridge.
void test_pair_does_not_measure_the_other_node() {
  PairNetwork* net = buildPair();
  net->world.link({"bridge-node", "valve-node"});
  net->far->blocked = true;
  net->hub->sendDownlink(kFarEui, bytesOf("set point"), seconds(0.3));
  net->world.run(seconds(20.0));
  const tadogado::DeviceLink& atBridgeNode = linkOf(*net->bridgeNode, kFar);
  TEST_ASSERT_EQUAL(1, atBridgeNode.downlinks);
  TEST_ASSERT_EQUAL(0, atBridgeNode.retransmissions);
  TEST_ASSERT_EQUAL(1, linkOf(*net->bridgeNode, kBridge).frames);
  // The ACK that the bridge node sent for the valve.
  TEST_ASSERT_EQUAL(0, linkOf(*net->valveNode, kFar).frames);
  TEST_ASSERT_EQUAL(0, net->valveNode->monitor.counters().verifiedAcks);
  delete net;
}

// A network slower than an echo is remembered.
void test_pair_sends_a_broadcast_on_only_once() {
  PairNetwork* net = buildPair(seconds(4.0));
  net->world.link({"bridge-node", "valve-node"});
  net->world.transmit(*net->far, broadcastFrame(5, kFarEui, 1));
  net->hub->sendAdvertisement();
  net->world.run(seconds(30.0));
  TEST_ASSERT_EQUAL(1, countClass(net->world.sentBy("bridge-node"), FrameClass::Broadcast));
  TEST_ASSERT_EQUAL(1, countClass(net->world.sentBy("valve-node"), FrameClass::Broadcast));
  TEST_ASSERT_EQUAL(1, linkOf(*net->valveNode, kFar).frames);
  TEST_ASSERT_EQUAL(1, linkOf(*net->bridgeNode, kBridge).frames);
  delete net;
}

void test_airtime_is_counted_per_valve() {
  Network* net = build(true);
  net->far->sendUplink(bytesOf("report"));
  net->hub->sendDownlink(kFarEui, bytesOf("set point"), seconds(0.3));
  net->world.run(seconds(9.0));
  const uint64_t now = net->proxy->nowUs();
  const uint64_t used = net->proxy->engine.duty().usedUs(now);
  TEST_ASSERT_GREATER_THAN(0, used);
  TEST_ASSERT_EQUAL_UINT64(used, linkOf(*net->proxy, kFar).airtime.usedUs(now));
  TEST_ASSERT_EQUAL_UINT64(0, linkOf(*net->proxy, kBridge).airtime.usedUs(now));
  TEST_ASSERT_EQUAL_UINT64(0, linkOf(*net->proxy, kFar).airtime.usedUs(now + seconds(3900.0)));
  delete net;
}

void test_pair_bridge_node_promises_only_what_the_valve_node_can_deliver() {
  PairNetwork* net = buildPair();
  net->valveNode->engine.duty().record(tadogado::DutyCycleLimiter::kBudgetUs - seconds(0.1), 1);
  net->hub->sendDownlink(kFarEui, bytesOf("set point"), seconds(0.3));
  net->world.run(seconds(12.0));
  TEST_ASSERT_EQUAL(0, net->far->received.size());
  TEST_ASSERT_GREATER_THAN(0, net->bridgeNode->engine.counters().acksWithheld);
  TEST_ASSERT_EQUAL(1, net->hub->downlinkResults.size());
  TEST_ASSERT_FALSE(net->hub->downlinkResults[0].acked);
  TEST_ASSERT_EQUAL(0, net->valveNode->engine.counters().peerFramesDropped);
  delete net;
}

void test_pair_does_not_learn_a_sample_time_from_the_other_node() {
  PairNetwork* net = buildPair(20000);
  net->world.link({"bridge-node", "valve-node"});
  net->hub->sendDownlink(kFarEui, bytesOf("first"), seconds(0.3));
  while (net->valveNode->monitor.counters().framesRx == 0) {
    net->world.run(seconds(0.01));
  }
  net->world.run(seconds(0.005));
  TEST_ASSERT_FALSE(net->valveNode->engine.anchors().contains(kFar.data()));
  delete net;
}

// The far valve hears the bridge too. The valve node hears it answer the bridge itself,
// and counts the frame it then has to deliver all the same. It hears the bridge node
// 20 dB weaker than the valve, so it can tell their answers apart once it has heard
// the bridge node answer. The frame comes over the network after that answer, as on
// WiFi; were it quicker, the valve node would be sending its train by then.
void test_pair_counts_frames_the_valve_already_had() {
  PairNetwork* net = buildPair(20000);
  net->world.link({"bridge-node", "valve-node"});
  net->valveNode->rssiFrom["far"] = -60.0f;
  net->far->sendUplink(bytesOf("report"));  // So the valve node knows its signal.
  net->world.run(seconds(6.0));
  net->hub->sendDownlink(kFarEui, bytesOf("set point"), seconds(0.3));  // Only through the pair.
  net->world.run(seconds(4.0));
  float peerRssi = 0.0f;
  TEST_ASSERT_TRUE(net->valveNode->engine.peerAnswerRssi(peerRssi));
  TEST_ASSERT_EQUAL_FLOAT(-80.0f, peerRssi);
  TEST_ASSERT_TRUE(net->valveNode->engine.canTellOwnAnswer(kFar.data()));
  TEST_ASSERT_EQUAL(0, linkOf(*net->valveNode, kFar).alreadyAnswered);
  net->world.link({"bridge", "far"});
  net->hub->sendDownlink(kFarEui, bytesOf("set point 2"), seconds(0.3));
  net->world.run(seconds(4.0));
  TEST_ASSERT_GREATER_OR_EQUAL(1, linkOf(*net->valveNode, kFar).alreadyAnswered);
  TEST_ASSERT_EQUAL(0, linkOf(*net->bridgeNode, kFar).alreadyAnswered);
  delete net;
}

// The valve node hears the far valve and the bridge node equally strong. The bridge
// node's answer for the valve then looks like the valve's own: nothing may be counted.
void test_pair_counts_nothing_it_cannot_tell_apart() {
  PairNetwork* net = buildPair(20000);
  net->world.link({"bridge-node", "valve-node"});
  net->far->sendUplink(bytesOf("report"));
  net->world.run(seconds(6.0));
  for (const char* command : {"set point", "set point 2"}) {
    net->hub->sendDownlink(kFarEui, bytesOf(command), seconds(0.3));
    net->world.run(seconds(4.0));
  }
  TEST_ASSERT_EQUAL(2, net->far->received.size());
  TEST_ASSERT_EQUAL(0, linkOf(*net->valveNode, kFar).alreadyAnswered);
  TEST_ASSERT_FALSE(net->valveNode->engine.canTellOwnAnswer(kFar.data()));
  delete net;
}

void test_pair_counts_nothing_when_the_valve_only_hears_the_proxy() {
  PairNetwork* net = buildPair();
  net->far->sendUplink(bytesOf("report"));
  net->world.run(seconds(2.0));
  net->hub->sendDownlink(kFarEui, bytesOf("set point"), seconds(0.3));
  net->world.run(seconds(4.0));
  TEST_ASSERT_EQUAL(1, net->far->received.size());
  TEST_ASSERT_EQUAL(0, linkOf(*net->valveNode, kFar).alreadyAnswered);
  delete net;
}

// Proxied toward the valve only: its own frames are left to the bridge, which here
// cannot hear them.
void test_proxy_toward_the_valve_only_leaves_its_frames_alone() {
  Network* net = build(true, {});
  net->proxy->monitor.setProxied(kFar.data(), true, tadogado::ProxyDirection::ToValve);
  net->far->sendUplink(bytesOf("report"));
  net->hub->sendDownlink(kFarEui, bytesOf("set point"), seconds(0.3));
  net->world.run(seconds(9.0));
  TEST_ASSERT_EQUAL(0, net->hub->uplinks.size());
  assertUplinks(*net->far, {{10, false}});
  TEST_ASSERT_EQUAL(1, net->far->received.size());
  TEST_ASSERT_TRUE(net->hub->downlinkResults[0].acked);
  TEST_ASSERT_EQUAL(0, linkOf(*net->proxy, kFar).proxiedUplinks);
  TEST_ASSERT_EQUAL(1, linkOf(*net->proxy, kFar).proxiedDownlinks);
  delete net;
}

// Proxied toward the bridge only: the bridge's frames are left to the valve, which here
// cannot hear them, and no Router Advertisement is sent on.
void test_proxy_toward_the_bridge_only_leaves_the_bridges_frames_alone() {
  Network* net = build(true, {});
  net->proxy->monitor.setProxied(kFar.data(), true, tadogado::ProxyDirection::ToBridge);
  net->far->sendUplink(bytesOf("report"));
  net->hub->sendDownlink(kFarEui, bytesOf("set point"), seconds(0.3));
  net->hub->sendAdvertisement();
  net->world.run(seconds(9.0));
  TEST_ASSERT_EQUAL(1, net->hub->uplinks.size());
  assertUplinks(*net->far, {{10, true}});
  TEST_ASSERT_EQUAL(0, net->far->received.size());
  TEST_ASSERT_FALSE(net->hub->downlinkResults[0].acked);
  TEST_ASSERT_EQUAL(0, net->far->advertisements);
  TEST_ASSERT_EQUAL(1, linkOf(*net->proxy, kFar).proxiedUplinks);
  TEST_ASSERT_EQUAL(0, linkOf(*net->proxy, kFar).proxiedDownlinks);
  TEST_ASSERT_EQUAL(0, net->proxy->engine.counters().advertisementsForwarded);
  delete net;
}

}  // namespace

void setUp() {}
void tearDown() {}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_without_a_proxy_the_far_valve_gets_nothing_through);
  RUN_TEST(test_monitor_mode_never_transmits);
  RUN_TEST(test_monitor_mode_measures_the_links);
  RUN_TEST(test_uplink_of_a_far_valve_reaches_the_bridge_in_one_try);
  RUN_TEST(test_proxied_ack_lands_in_the_documented_window);
  RUN_TEST(test_downlink_reaches_a_sleeping_valve_byte_for_byte);
  RUN_TEST(test_second_downlink_uses_a_short_train);
  RUN_TEST(test_retry_after_a_lost_ack_is_answered_but_not_delivered_twice);
  RUN_TEST(test_devices_that_were_not_named_are_left_alone);
  RUN_TEST(test_no_proxied_ack_with_an_empty_budget);
  RUN_TEST(test_refused_frame_is_taken_when_budget_returns_for_its_retry);
  RUN_TEST(test_transmissions_stay_within_the_budget);
  RUN_TEST(test_router_advertisement_reaches_the_far_valve_at_a_limited_rate);
  RUN_TEST(test_bridge_is_learned_from_two_trains);
  RUN_TEST(test_a_valve_can_be_found_by_its_downlink);
  RUN_TEST(test_status_holds_real_proxy_numbers);
  RUN_TEST(test_pair_serves_both_directions);
  RUN_TEST(test_pair_stays_available_while_the_other_node_sends_a_train);
  RUN_TEST(test_pair_nodes_ignore_each_others_transmissions);
  RUN_TEST(test_pair_bridge_node_does_not_answer_the_valve_node);
  RUN_TEST(test_pair_valve_node_does_not_answer_the_bridge_node);
  RUN_TEST(test_pair_does_not_measure_the_other_node);
  RUN_TEST(test_pair_sends_a_broadcast_on_only_once);
  RUN_TEST(test_airtime_is_counted_per_valve);
  RUN_TEST(test_pair_bridge_node_promises_only_what_the_valve_node_can_deliver);
  RUN_TEST(test_pair_does_not_learn_a_sample_time_from_the_other_node);
  RUN_TEST(test_pair_counts_frames_the_valve_already_had);
  RUN_TEST(test_pair_counts_nothing_it_cannot_tell_apart);
  RUN_TEST(test_pair_counts_nothing_when_the_valve_only_hears_the_proxy);
  RUN_TEST(test_proxy_toward_the_valve_only_leaves_its_frames_alone);
  RUN_TEST(test_proxy_toward_the_bridge_only_leaves_the_bridges_frames_alone);
  return UNITY_END();
}
