#include "dedup.h"

#include "frames.h"

namespace tadogado {

namespace {
constexpr uint64_t kEchoTtlUs = 3000000;
constexpr uint64_t kRetransmitTtlUs = 10000000;
}  // namespace

Verdict FrameDeduplicator::classify(const uint8_t* frame, size_t length, uint64_t nowUs) {
  const uint64_t exact = hashBytes(frame, length);
  if (echoes_.contains(exact, nowUs)) {
    statsEchoes++;
    return Verdict::Echo;
  }
  echoes_.add(exact, nowUs + kEchoTtlUs);

  const uint64_t datagram = retransmissionHash(frame, length);
  const bool known = datagrams_.contains(datagram, nowUs);
  datagrams_.add(datagram, nowUs + kRetransmitTtlUs);
  if (known) {
    statsRetransmissions++;
    return Verdict::Retransmission;
  }
  statsNew++;
  return Verdict::New;
}

void FrameDeduplicator::noteSent(const uint8_t* frame, size_t length, uint64_t nowUs) {
  echoes_.add(hashBytes(frame, length), nowUs + kEchoTtlUs);
  datagrams_.add(retransmissionHash(frame, length), nowUs + kRetransmitTtlUs);
}

}  // namespace tadogado
