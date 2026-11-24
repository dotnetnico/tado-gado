// SHA-256 and HMAC, for the link between two nodes. Self-contained, so the native
// tests run the same code as the firmware.
#pragma once

#include <cstddef>
#include <cstdint>

namespace tadogado {

constexpr size_t kSha256Len = 32;

void sha256(const uint8_t* data, size_t length, uint8_t out[kSha256Len]);

// `length` may be at most kHmacMaxMessage. A longer message is refused with an
// all-zero result rather than overflow; pair_protocol checks the bound at compile time.
constexpr size_t kHmacMaxMessage = 256;
void hmacSha256(const uint8_t* key, size_t keyLength, const uint8_t* data, size_t length, uint8_t out[kSha256Len]);

// PBKDF2-HMAC-SHA256 (RFC 8018) with one output block. `saltLength` may be at most
// kHmacMaxMessage - 4.
void pbkdf2Sha256(const uint8_t* password, size_t passwordLength, const uint8_t* salt, size_t saltLength,
                  uint32_t iterations, uint8_t out[kSha256Len]);

// Compares in a time that depends only on `length`.
bool equalConstantTime(const uint8_t* a, const uint8_t* b, size_t length);

}  // namespace tadogado
