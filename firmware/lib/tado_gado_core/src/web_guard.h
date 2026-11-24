// Checks on an HTTP request, apart from the server, so the native tests can run them.
#pragma once

#include <cstddef>
#include <cstdint>

namespace tadogado {

// A DNS rebinding page reaches the node under its own host name, so only the names of
// the node are served: its IP address, its mDNS name, and that name under the DNS
// suffix of a router, such as tado-gado-xxxx.lan. The browser sets Host, and the page
// cannot change it. `name` is in lower case.
bool hostAllowed(const char* host, const char* address, const char* name);

// Makes a client that sent a wrong token wait, for longer each time. It counts per
// client, so that one client cannot lock out another, who may want to stop the proxy.
class TokenLockout {
 public:
  static constexpr size_t kClients = 8;
  static constexpr uint32_t kFirstDelayMs = 500;
  static constexpr uint32_t kMaxDelayMs = 30000;

  bool locked(uint32_t client, uint64_t nowMs) const;
  void failed(uint32_t client, uint64_t nowMs);
  void succeeded(uint32_t client);

 private:
  struct Entry {
    bool used;
    uint32_t client;
    uint32_t delayMs;
    uint64_t untilMs;
  };

  const Entry* find(uint32_t client) const;

  Entry entries_[kClients] = {};
};

}  // namespace tadogado
