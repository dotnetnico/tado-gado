#include "web.h"

#include <WiFi.h>
#include <esp_http_server.h>
#include <esp_timer.h>
#include <lwip/sockets.h>

#include <cstring>

#include "generated/dashboard_html.h"
#include "link_monitor.h"
#include "radio_task.h"
#include "settings.h"
#include "sha256.h"
#include "status_json.h"
#include "update.h"
#include "web_guard.h"

// The server of ESP-IDF rather than the Arduino WebServer: it parses a request in fixed
// buffers and leaves the body to the handler, so a request of any size or shape cannot
// take the heap. All handlers run in its one task, so they share static buffers.

namespace {

constexpr size_t kStatusBufferBytes = tadogado::kStatusJsonCapacity;
constexpr int kMaxChannel = 34;
// The largest valid body is about 60 bytes. A bigger one is refused unread.
constexpr size_t kMaxBodyBytes = 512;
constexpr uint64_t kBodyDeadlineUs = 3000000;
constexpr size_t kUpdateChunkBytes = 4096;  // A flash sector.
constexpr uint64_t kUpdateDeadlineUs = 180000000;
constexpr size_t kMaxTokenBytes = 64;
constexpr size_t kMaxHostBytes = 80;
constexpr uint32_t kServerStackBytes = 8192;
constexpr BaseType_t kServerCore = 0;  // With Wi-Fi, so core 1 stays free for the radio.
constexpr UBaseType_t kServerPriority = 1;
constexpr uint16_t kReceiveTimeoutSeconds = 2;
// A flash write stalls both cores, so it waits for a quiet radio, but not forever.
constexpr uint64_t kMaxSaveDelayUs = 60000000;

bool hasToken = false;
uint8_t tokenDigest[tadogado::kSha256Len];
String hostName;
tadogado::TokenLockout lockout;
char statusBuffer[kStatusBufferBytes];

// Settings to write to flash once the radio is idle. The server's task adds to them,
// and the web task writes them.
struct PendingSaves {
  bool radio = false;
  uint8_t channel = 0;
  int8_t fsctrl0 = 0;
  bool power = false;
  uint8_t pa = 0;
  bool proxied = false;
  bool names = false;
  uint64_t sinceUs = 0;

  bool any() const { return radio || power || proxied || names; }
};

PendingSaves pending;
SemaphoreHandle_t pendingLock;

template <typename Change>
void addPending(Change change) {
  xSemaphoreTake(pendingLock, portMAX_DELAY);
  if (!pending.any()) {
    pending.sinceUs = esp_timer_get_time();
  }
  change(pending);
  xSemaphoreGive(pendingLock);
}

esp_err_t reply(httpd_req_t* request, const char* status, const char* type, const char* content) {
  httpd_resp_set_status(request, status);
  httpd_resp_set_type(request, type);
  httpd_resp_set_hdr(request, "X-Content-Type-Options", "nosniff");
  httpd_resp_set_hdr(request, "X-Frame-Options", "DENY");
  httpd_resp_set_hdr(request, "Content-Security-Policy", "frame-ancestors 'none'");
  httpd_resp_set_hdr(request, "Cache-Control", "no-store");
  return httpd_resp_send(request, content, HTTPD_RESP_USE_STRLEN);
}

esp_err_t replyText(httpd_req_t* request, const char* status, const char* text) {
  return reply(request, status, "text/plain", text);
}

esp_err_t replyJson(httpd_req_t* request, const char* json) {
  return reply(request, "200 OK", "application/json", json);
}

bool fromThisNode(httpd_req_t* request) {
  char host[kMaxHostBytes];
  return httpd_req_get_hdr_value_str(request, "Host", host, sizeof(host)) == ESP_OK &&
         tadogado::hostAllowed(host, WiFi.localIP().toString().c_str(), hostName.c_str());
}

// The IPv4 address, or the last part of an IPv6 address.
uint32_t clientOf(httpd_req_t* request) {
  sockaddr_storage address = {};
  socklen_t length = sizeof(address);
  uint32_t client = 0;
  if (lwip_getpeername(httpd_req_to_sockfd(request), reinterpret_cast<sockaddr*>(&address), &length) != 0) {
    return client;
  }
  if (address.ss_family == AF_INET6) {
    const auto* v6 = reinterpret_cast<const sockaddr_in6*>(&address);
    memcpy(&client, reinterpret_cast<const uint8_t*>(&v6->sin6_addr) + 12, sizeof(client));
  } else {
    client = reinterpret_cast<const sockaddr_in*>(&address)->sin_addr.s_addr;
  }
  return client;
}

// Sends the error response itself. Compares SHA-256 digests, so neither the content nor
// the length of the token shows in the response time.
bool authorized(httpd_req_t* request) {
  const uint64_t nowMs = esp_timer_get_time() / 1000;
  const uint32_t client = clientOf(request);
  if (lockout.locked(client, nowMs)) {
    replyText(request, "429 Too Many Requests", "too many wrong tokens: wait a moment");
    return false;
  }
  // A request without a token is no guess. A web page can make a browser send those,
  // and must not lock its user out.
  char token[kMaxTokenBytes + 1] = {};
  const esp_err_t read = httpd_req_get_hdr_value_str(request, "X-Admin-Token", token, sizeof(token));
  if (read != ESP_OK && read != ESP_ERR_HTTPD_RESULT_TRUNC) {
    replyText(request, "403 Forbidden", "admin token required");
    return false;
  }
  uint8_t given[tadogado::kSha256Len];
  tadogado::sha256(reinterpret_cast<const uint8_t*>(token), strlen(token), given);
  if (!hasToken || read != ESP_OK || !tadogado::equalConstantTime(given, tokenDigest, sizeof(given))) {
    lockout.failed(client, nowMs);
    replyText(request, "403 Forbidden", "admin token required");
    return false;
  }
  lockout.succeeded(client);
  return true;
}

// Reads the whole body, which is known to fit. A client that sends it too slowly fails.
bool readBody(httpd_req_t* request, char* out) {
  const uint64_t deadline = esp_timer_get_time() + kBodyDeadlineUs;
  size_t length = 0;
  while (length < request->content_len) {
    const int received = httpd_req_recv(request, out + length, request->content_len - length);
    if (received <= 0 || static_cast<uint64_t>(esp_timer_get_time()) > deadline) {
      return false;
    }
    length += static_cast<size_t>(received);
  }
  out[length] = '\0';
  return true;
}

// `json` is empty for a GET request, and for an upload, which reads its own body.
struct Endpoint {
  const char* path;
  httpd_method_t method;
  esp_err_t (*handle)(httpd_req_t* request, const char* json);
  bool upload;
};

esp_err_t handleIndex(httpd_req_t* request, const char*) {
  return reply(request, "200 OK", "text/html", kDashboardHtml);
}

esp_err_t handleStatus(httpd_req_t* request, const char*) {
  if (!radio::writeStatus(statusBuffer, sizeof(statusBuffer), !hasToken)) {
    return replyText(request, "500 Internal Server Error", "status too large");
  }
  return replyJson(request, statusBuffer);
}

esp_err_t handleHealth(httpd_req_t* request, const char*) { return replyJson(request, "{\"status\":\"ok\"}"); }

esp_err_t handleSetRadio(httpd_req_t* request, const char* json) {
  int channel = 0;
  int fsctrl0 = 0;
  const bool hasChannel =
      tadogado::jsonIntField(json, "channel", channel) && tadogado::jsonIntField(json, "fsctrl0", fsctrl0);
  if (hasChannel && (channel < 0 || channel > kMaxChannel || fsctrl0 < -128 || fsctrl0 > 127)) {
    return replyText(request, "400 Bad Request", "need channel 0-34 and fsctrl0 -128 to 127");
  }
  // Output power, as a PATABLE value. Lower it for an antenna with gain.
  int pa = 0;
  const bool hasPower = tadogado::jsonIntField(json, "pa", pa);
  if (hasPower && (pa < 0 || pa > 255)) {
    return replyText(request, "400 Bad Request", "need pa 0 to 255");
  }
  if (hasChannel) {
    radio::requestRetune(static_cast<uint8_t>(channel), static_cast<int8_t>(fsctrl0));
    addPending([channel, fsctrl0](PendingSaves& saves) {
      saves.radio = true;
      saves.channel = static_cast<uint8_t>(channel);
      saves.fsctrl0 = static_cast<int8_t>(fsctrl0);
    });
  }
  if (hasPower) {
    radio::requestPower(static_cast<uint8_t>(pa));
    addPending([pa](PendingSaves& saves) {
      saves.power = true;
      saves.pa = static_cast<uint8_t>(pa);
    });
  }
  return replyJson(request, "{\"saved\":true}");
}

esp_err_t handleSurvey(httpd_req_t* request, const char*) {
  if (!radio::requestSurvey()) {
    return replyText(request, "409 Conflict", "switch to monitor mode first: a survey retunes the radio");
  }
  return replyJson(request, "{\"started\":true}");
}

esp_err_t handleSetMode(httpd_req_t* request, const char* json) {
  char mode[16];
  if (!tadogado::jsonStringField(json, "mode", mode, sizeof(mode)) ||
      (strcmp(mode, "monitor") != 0 && strcmp(mode, "proxy") != 0)) {
    return replyText(request, "400 Bad Request", "mode must be monitor or proxy");
  }
  if (update::restarting()) {
    return replyText(request, "409 Conflict", "the node is about to restart");
  }
  if (!radio::requestProxyMode(strcmp(mode, "proxy") == 0)) {
    return replyText(request, "409 Conflict",
                     "the two nodes have no roles yet: at least one of them must hear the bridge");
  }
  return replyJson(request, "{\"accepted\":true}");
}

esp_err_t handleSetProxy(httpd_req_t* request, const char* json) {
  char device[16];
  uint8_t prefix[tadogado::kPrefixLen];
  bool enabled = false;
  if (!tadogado::jsonStringField(json, "device", device, sizeof(device)) || !tadogado::parsePrefix(device, prefix) ||
      !tadogado::jsonBoolField(json, "enabled", enabled)) {
    return replyText(request, "400 Bad Request", "need a 5-byte device and a boolean enabled");
  }
  // "both" when absent, as before v0.9.0.
  char direction[12] = "both";
  if (strstr(json, "\"direction\"") != nullptr &&
      !tadogado::jsonStringField(json, "direction", direction, sizeof(direction))) {
    direction[0] = '\0';
  }
  tadogado::ProxyDirection proxyDirection = tadogado::ProxyDirection::Both;
  if (strcmp(direction, "downlink") == 0) {
    proxyDirection = tadogado::ProxyDirection::ToValve;
  } else if (strcmp(direction, "uplink") == 0) {
    proxyDirection = tadogado::ProxyDirection::ToBridge;
  } else if (strcmp(direction, "both") != 0) {
    return replyText(request, "400 Bad Request", "direction is \"both\", \"downlink\" or \"uplink\"");
  }
  if (!radio::setProxied(prefix, enabled, proxyDirection)) {
    return replyText(request, "409 Conflict", "the proxy serves at most 16 devices");
  }
  addPending([](PendingSaves& saves) { saves.proxied = true; });
  return replyJson(request, "{\"saved\":true}");
}

esp_err_t handleSetName(httpd_req_t* request, const char* json) {
  char device[16];
  char name[tadogado::kMaxNameBytes + 1];
  uint8_t prefix[tadogado::kPrefixLen];
  if (!tadogado::jsonStringField(json, "device", device, sizeof(device)) || !tadogado::parsePrefix(device, prefix) ||
      !tadogado::jsonStringField(json, "name", name, sizeof(name))) {
    return replyText(request, "400 Bad Request", "need a 5-byte device and a name of at most 24 bytes");
  }
  if (!radio::setName(prefix, name)) {
    return replyText(request, "400 Bad Request", "the name has an invalid character, or the name table is full");
  }
  addPending([](PendingSaves& saves) { saves.names = true; });
  return replyJson(request, "{\"saved\":true}");
}

// Fills `out` unless the body ends first. Returns the length, or 0 if the client stalls.
size_t receiveChunk(httpd_req_t* request, uint8_t* out, size_t wanted, uint64_t deadlineUs) {
  size_t length = 0;
  while (length < wanted) {
    const int received = httpd_req_recv(request, reinterpret_cast<char*>(out) + length, wanted - length);
    if (static_cast<uint64_t>(esp_timer_get_time()) > deadlineUs) {
      return 0;
    }
    if (received == HTTPD_SOCK_ERR_TIMEOUT) {
      continue;
    }
    if (received <= 0) {
      return 0;
    }
    length += static_cast<size_t>(received);
  }
  return length;
}

const char* receiveUpdate(httpd_req_t* request) {
  static uint8_t chunk[kUpdateChunkBytes];
  const uint64_t deadline = static_cast<uint64_t>(esp_timer_get_time()) + kUpdateDeadlineUs;
  const char* error = update::begin(request->content_len);
  size_t left = request->content_len;
  while (error == nullptr && left > 0) {
    const size_t wanted = left < sizeof(chunk) ? left : sizeof(chunk);
    if (receiveChunk(request, chunk, wanted, deadline) != wanted) {
      update::abort();
      return "the upload stopped";
    }
    error = update::write(chunk, wanted);
    left -= wanted;
    vTaskDelay(1);  // The idle task feeds the watchdog.
  }
  return error != nullptr ? error : update::finish();
}

// An empty body only asks whether an update can start. The dashboard asks first, since a
// browser that is still sending does not show the answer.
esp_err_t handleUpdate(httpd_req_t* request, const char*) {
  // A flash write stalls both cores, and the proxy would miss its ACKs.
  const char* blocked = radio::proxyMode() ? "switch to monitor mode first" : update::blocked();
  if (blocked != nullptr) {
    replyText(request, "409 Conflict", blocked);
    return ESP_FAIL;
  }
  if (request->content_len == 0) {
    return replyJson(request, "{\"ready\":true}");
  }
  const char* error = receiveUpdate(request);
  if (error != nullptr) {
    replyText(request, "400 Bad Request", error);
    return ESP_FAIL;
  }
  Serial.printf("Firmware %s is installed: restarting.\n", update::version());
  return replyJson(request, "{\"installed\":true}");
}

const Endpoint kEndpoints[] = {
    {"/", HTTP_GET, handleIndex, false},
    {"/api/status", HTTP_GET, handleStatus, false},
    {"/health", HTTP_GET, handleHealth, false},
    {"/api/radio", HTTP_POST, handleSetRadio, false},
    {"/api/survey", HTTP_POST, handleSurvey, false},
    {"/api/mode", HTTP_POST, handleSetMode, false},
    {"/api/proxy", HTTP_POST, handleSetProxy, false},
    {"/api/name", HTTP_POST, handleSetName, false},
    {"/api/update", HTTP_POST, handleUpdate, true},
};

// Every request passes here. ESP_FAIL makes the server close the connection, so it does
// not read what the client still has to send.
esp_err_t dispatch(httpd_req_t* request) {
  static char body[kMaxBodyBytes + 1];
  const Endpoint& endpoint = *static_cast<const Endpoint*>(request->user_ctx);
  if (endpoint.upload) {
    // Nothing of a large body is read before the token is known to be right.
    if (!fromThisNode(request)) {
      replyText(request, "421 Misdirected Request", "unknown host name");
      return ESP_FAIL;
    }
    return authorized(request) ? endpoint.handle(request, "") : ESP_FAIL;
  }
  if (request->content_len > kMaxBodyBytes) {
    replyText(request, "413 Payload Too Large", "body too large");
    return ESP_FAIL;
  }
  if (!readBody(request, body)) {
    return ESP_FAIL;
  }
  if (!fromThisNode(request)) {
    return replyText(request, "421 Misdirected Request", "unknown host name");
  }
  if (endpoint.method == HTTP_POST && !authorized(request)) {
    return ESP_OK;
  }
  return endpoint.handle(request, body);
}

void flushPending(bool now) {
  xSemaphoreTake(pendingLock, portMAX_DELAY);
  const bool due = pending.any() && (now || radio::idle() || esp_timer_get_time() - pending.sinceUs > kMaxSaveDelayUs);
  const PendingSaves saves = due ? pending : PendingSaves{};
  if (due) {
    pending = PendingSaves{};
  }
  xSemaphoreGive(pendingLock);

  if (saves.radio) {
    saveRadioSettings(saves.channel, saves.fsctrl0);
  }
  if (saves.power) {
    savePower(saves.pa);
  }
  if (saves.proxied) {
    uint8_t devices[tadogado::kMaxProxied * tadogado::kPrefixLen];
    uint8_t directions[tadogado::kMaxProxied];
    const size_t count = radio::proxiedDevices(devices, directions);
    saveProxied(devices, directions, count);
  }
  if (saves.names) {
    static tadogado::NameTable::Entry names[tadogado::NameTable::kCapacity];
    radio::copyNames(names);
    saveNames(names);
  }
}

}  // namespace

namespace web {

bool begin(const String& adminToken, const String& name) {
  hasToken = !adminToken.isEmpty();
  tadogado::sha256(reinterpret_cast<const uint8_t*>(adminToken.c_str()), adminToken.length(), tokenDigest);
  hostName = name;
  hostName.toLowerCase();
  pendingLock = xSemaphoreCreateMutex();

  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.task_priority = kServerPriority;
  config.stack_size = kServerStackBytes;
  config.core_id = kServerCore;
  config.max_uri_handlers = sizeof(kEndpoints) / sizeof(kEndpoints[0]);
  config.recv_wait_timeout = kReceiveTimeoutSeconds;
  config.lru_purge_enable = true;  // Idle connections make way for a new one.
  httpd_handle_t server = nullptr;
  if (httpd_start(&server, &config) != ESP_OK) {
    return false;
  }
  for (const Endpoint& endpoint : kEndpoints) {
    httpd_uri_t uri = {};
    uri.uri = endpoint.path;
    uri.method = endpoint.method;
    uri.handler = dispatch;
    uri.user_ctx = const_cast<Endpoint*>(&endpoint);
    if (httpd_register_uri_handler(server, &uri) != ESP_OK) {
      return false;
    }
  }
  return true;
}

void loop() {
  if (radio::takeProxiedChanged()) {
    addPending([](PendingSaves& saves) { saves.proxied = true; });
  }
  if (radio::takeNamesChanged()) {
    addPending([](PendingSaves& saves) { saves.names = true; });
  }
  update::loop();
  const bool restart = update::restartDue();
  flushPending(restart);
  if (restart) {
    ESP.restart();
  }
}

}  // namespace web
