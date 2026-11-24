// Tado-gado firmware: measures the Tado radio links, and can proxy for far devices.
//
// First boot opens the Wi-Fi network "tado-gado-setup" to enter your Wi-Fi and an admin
// token. Once that is saved the setup network never opens again, so a stranger
// cannot use it to change the token; holding BOOT for 5 s just after a start erases
// all settings and opens it once more. Every boot starts in monitor mode, where
// nothing is transmitted.
#include <Arduino.h>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <WiFiManager.h>

#include "board.h"
#include "radio_task.h"
#include "reset_button.h"
#include "settings.h"
#include "web.h"

namespace {

constexpr const char* kHostName = "tado-gado";
constexpr const char* kSetupNetwork = "tado-gado-setup";
constexpr int kTokenMaxLength = 64;
// The token guards the radio, and a captured pair message lets an attacker test
// guesses offline, so it must not be short.
constexpr unsigned kTokenMinLength = 16;
constexpr unsigned kSuggestedTokenLength = 20;
constexpr unsigned long kSetupTimeoutSeconds = 300;
constexpr uint32_t kWebStackBytes = 8192;
constexpr uint32_t kWebIntervalMs = 100;
constexpr BaseType_t kWebCore = 0;  // With Wi-Fi, so core 1 stays free for the radio.
constexpr UBaseType_t kWebPriority = 1;
constexpr uint32_t kResetStackBytes = 4096;
constexpr uint32_t kResetIntervalMs = 20;
constexpr UBaseType_t kResetPriority = 1;

// The HTTP server has its own task. This one writes settings to flash.
void webTask(void*) {
  for (;;) {
    web::loop();
    vTaskDelay(pdMS_TO_TICKS(kWebIntervalMs));
  }
}

// Has its own task, because setup() waits for as long as the saved Wi-Fi is out of
// reach, and that is when a factory reset is needed. It erases after the restart, when
// no other task can save a setting again.
void resetButtonTask(void*) {
  pinMode(board::kPinBoot, INPUT_PULLUP);
  tadogado::ResetButton button;
  while (!button.done(millis())) {
    if (button.update(digitalRead(board::kPinBoot) == LOW, millis())) {
      Serial.println("BOOT was held: erasing all settings.");
      requestErase();
      ESP.restart();
    }
    vTaskDelay(pdMS_TO_TICKS(kResetIntervalMs));
  }
  vTaskDelete(nullptr);
}

// Two nodes must not claim the same name on the network, so it ends in the MAC address.
String hostName() {
  uint8_t mac[6];
  WiFi.macAddress(mac);
  char name[16];
  snprintf(name, sizeof(name), "%s-%02x%02x", kHostName, mac[4], mac[5]);
  return name;
}

// A random token to suggest in the setup page, without characters that look alike.
String suggestedToken() {
  static const char kAlphabet[] = "abcdefghjkmnpqrstuvwxyz23456789";
  String token;
  for (unsigned i = 0; i < kSuggestedTokenLength; i++) {
    token += kAlphabet[esp_random() % (sizeof(kAlphabet) - 1)];
  }
  return token;
}

// Blocks until connected, but the radio task already runs, so the node measures
// from the first second.
//
// The setup network is open, so it only opens while no Wi-Fi is saved: on first boot
// and after a factory reset. A saved network that is out of reach is retried for as
// long as it takes. The token can only be chosen there, so changing it needs the
// factory reset, and so physical access.
String connectWifi(bool erase, const String& currentToken) {
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(hostName().c_str());

  WiFiManager manager;
  if (erase) {
    manager.resetSettings();
    eraseDone();
  }
  if (manager.getWiFiIsSaved()) {
    WiFi.begin();
    uint32_t waited = 0;
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      if (++waited % 20 == 0) {
        Serial.println("Waiting for the saved Wi-Fi network...");
      }
    }
    return currentToken;
  }

  // Only the Wi-Fi page and Exit. WiFiManager serves a firmware upload at /u whatever
  // the menu shows, on a network anyone can join, so these routes are taken first.
  const char* menu[] = {"wifi", "exit"};
  manager.setMenu(menu, 2);
  manager.setWebServerCallback([&manager]() {
    for (const char* path : {"/u", "/update", "/erase", "/info"}) {
      manager.server->on(path, [&manager]() { manager.server->send(403, "text/plain", "not available"); });
    }
  });
  const String suggestion = suggestedToken();
  WiFiManagerParameter tokenField("token", "Admin token, at least 16 characters. Write it down.",
                                  currentToken.isEmpty() ? suggestion.c_str() : "", kTokenMaxLength);
  if (currentToken.isEmpty()) {
    manager.addParameter(&tokenField);
  }
  manager.setBreakAfterConfig(true);
  manager.setConfigPortalTimeout(kSetupTimeoutSeconds);
  if (!manager.autoConnect(kSetupNetwork)) {
    // Nobody used it in time, or the Wi-Fi details did not work. WiFiManager saved those
    // all the same, and the next boot would wait for that network without a token. With
    // no Wi-Fi saved, the setup network opens again.
    manager.resetSettings();
    ESP.restart();
  }

  if (!currentToken.isEmpty()) {
    return currentToken;
  }
  String entered = tokenField.getValue();
  entered.trim();
  if (entered.length() < kTokenMinLength) {
    Serial.println("The admin token was shorter than 16 characters, so none is set: the dashboard is read-only.");
    Serial.println("To set up again, press RESET, and then hold BOOT for 5 s.");
    return "";
  }
  return entered;
}

}  // namespace

void setup() {
  Serial.begin(115200);
  const bool erase = eraseRequested();
  xTaskCreatePinnedToCore(resetButtonTask, "reset", kResetStackBytes, nullptr, kResetPriority, nullptr, kWebCore);
  if (erase) {
    eraseSettings();
  }

  Settings settings = loadSettings();
  radio::start({settings.channel, settings.fsctrl0, settings.pa, settings.proxied, settings.proxiedCount,
                settings.names, settings.proxiedDirection});

  const String token = connectWifi(erase, settings.adminToken);
  if (token != settings.adminToken) {
    saveAdminToken(token);
  }

  WiFi.setSleep(false);  // Mains powered, and sleep would delay frames between nodes.
  const String name = hostName();
  MDNS.begin(name.c_str());
  MDNS.addService("http", "tcp", 80);
  if (!web::begin(token, name)) {
    Serial.println("The web server did not start.");
    ESP.restart();
  }
  uint8_t nodeId[6];
  WiFi.macAddress(nodeId);
  radio::startPairing(token.c_str(), nodeId, nextBootNumber());
  xTaskCreatePinnedToCore(webTask, "web", kWebStackBytes, nullptr, kWebPriority, nullptr, kWebCore);
  Serial.printf("Dashboard: http://%s.local/ or http://%s/\n", name.c_str(), WiFi.localIP().toString().c_str());
}

// The Arduino loop runs on core 1 with the radio. The web task replaces it.
void loop() { vTaskDelete(nullptr); }
