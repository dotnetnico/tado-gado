#include "update.h"

#include <esp_ota_ops.h>
#include <esp_timer.h>

#include <atomic>

#include "build_info.h"
#include "image_check.h"

// The Arduino core confirms new firmware before setup() runs, unless this returns true.
extern "C" bool verifyRollbackLater() { return true; }

namespace update {

namespace {

// New firmware works if it still serves the dashboard this long after its start.
constexpr uint32_t kConfirmAfterMs = 30000;
// Time for the answer to reach the browser.
constexpr uint32_t kRestartAfterMs = 1000;

uint32_t nowMs() { return static_cast<uint32_t>(esp_timer_get_time() / 1000); }

// Only the web server's task uses these. Other tasks use the atomics below.
const esp_partition_t* target = nullptr;
esp_ota_handle_t handle = 0;
bool open = false;
size_t remaining = 0;
tadogado::ImageCheck check(kVariant);
bool headerChecked = false;

std::atomic<bool> pendingVerify{false};
std::atomic<bool> restartPending{false};
std::atomic<uint32_t> restartSinceMs{0};

const char* fail(const char* reason) {
  abort();
  return reason;
}

}  // namespace

const char* blocked() {
  if (restarting()) {
    return "the node is about to restart";
  }
  if (pendingVerify) {
    return "wait until the firmware just installed is confirmed";
  }
  return nullptr;
}

const char* begin(size_t length) {
  abort();
  target = esp_ota_get_next_update_partition(nullptr);
  if (target == nullptr) {
    return "there is no room for a second firmware";
  }
  if (length > target->size) {
    return "the image is too large";
  }
  // Erases as it goes, so no step keeps the flash busy for long.
  if (esp_ota_begin(target, OTA_WITH_SEQUENTIAL_WRITES, &handle) != ESP_OK) {
    return "the update could not start";
  }
  open = true;
  remaining = length;
  headerChecked = false;
  check = tadogado::ImageCheck(kVariant);
  return nullptr;
}

const char* write(const uint8_t* data, size_t length) {
  if (!open || length > remaining) {
    return fail("the upload is out of step");
  }
  check.feed(data, length);
  remaining -= length;
  if (!headerChecked && check.headerComplete()) {
    headerChecked = true;
    const char* error = check.headerError();
    if (error != nullptr) {
      return fail(error);
    }
  }
  if (esp_ota_write(handle, data, length) != ESP_OK) {
    return fail("writing the flash failed");
  }
  return nullptr;
}

const char* finish() {
  if (!open || remaining != 0) {
    return fail("the upload is not complete");
  }
  const char* error = check.error();
  if (error != nullptr) {
    return fail(error);
  }
  open = false;  // esp_ota_end() frees the handle, whatever it returns.
  if (esp_ota_end(handle) != ESP_OK) {
    return "the image is damaged";
  }
  if (esp_ota_set_boot_partition(target) != ESP_OK) {
    return "the new firmware could not be selected";
  }
  restartSinceMs = nowMs();
  restartPending = true;
  return nullptr;
}

void abort() {
  if (open) {
    esp_ota_abort(handle);
    open = false;
  }
}

const char* version() { return check.version(); }

bool unconfirmed() { return pendingVerify; }

bool restarting() { return restartPending; }

bool restartDue() { return restartPending && nowMs() - restartSinceMs >= kRestartAfterMs; }

void loop() {
  static bool started = false;
  static uint32_t startedMs = 0;
  if (!started) {
    esp_ota_img_states_t state;
    pendingVerify = esp_ota_get_state_partition(esp_ota_get_running_partition(), &state) == ESP_OK &&
                    state == ESP_OTA_IMG_PENDING_VERIFY;
    startedMs = nowMs();
    started = true;
  }
  if (pendingVerify && nowMs() - startedMs >= kConfirmAfterMs) {
    esp_ota_mark_app_valid_cancel_rollback();
    pendingVerify = false;
  }
}

}  // namespace update
