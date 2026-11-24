#include "sha256.h"

#include <cstring>

namespace tadogado {

namespace {

constexpr size_t kBlockLen = 64;

constexpr uint32_t kRoundConstants[64] = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};

uint32_t rotateRight(uint32_t value, int bits) { return (value >> bits) | (value << (32 - bits)); }

void compress(uint32_t state[8], const uint8_t block[kBlockLen]) {
  uint32_t w[64];
  for (int i = 0; i < 16; i++) {
    w[i] = static_cast<uint32_t>(block[4 * i]) << 24 | static_cast<uint32_t>(block[4 * i + 1]) << 16 |
           static_cast<uint32_t>(block[4 * i + 2]) << 8 | static_cast<uint32_t>(block[4 * i + 3]);
  }
  for (int i = 16; i < 64; i++) {
    const uint32_t s0 = rotateRight(w[i - 15], 7) ^ rotateRight(w[i - 15], 18) ^ (w[i - 15] >> 3);
    const uint32_t s1 = rotateRight(w[i - 2], 17) ^ rotateRight(w[i - 2], 19) ^ (w[i - 2] >> 10);
    w[i] = w[i - 16] + s0 + w[i - 7] + s1;
  }

  uint32_t a = state[0], b = state[1], c = state[2], d = state[3];
  uint32_t e = state[4], f = state[5], g = state[6], h = state[7];
  for (int i = 0; i < 64; i++) {
    const uint32_t s1 = rotateRight(e, 6) ^ rotateRight(e, 11) ^ rotateRight(e, 25);
    const uint32_t choose = (e & f) ^ (~e & g);
    const uint32_t temp1 = h + s1 + choose + kRoundConstants[i] + w[i];
    const uint32_t s0 = rotateRight(a, 2) ^ rotateRight(a, 13) ^ rotateRight(a, 22);
    const uint32_t majority = (a & b) ^ (a & c) ^ (b & c);
    const uint32_t temp2 = s0 + majority;
    h = g;
    g = f;
    f = e;
    e = d + temp1;
    d = c;
    c = b;
    b = a;
    a = temp1 + temp2;
  }
  state[0] += a;
  state[1] += b;
  state[2] += c;
  state[3] += d;
  state[4] += e;
  state[5] += f;
  state[6] += g;
  state[7] += h;
}

}  // namespace

void sha256(const uint8_t* data, size_t length, uint8_t out[kSha256Len]) {
  uint32_t state[8] = {0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19};

  size_t offset = 0;
  for (; offset + kBlockLen <= length; offset += kBlockLen) {
    compress(state, data + offset);
  }

  uint8_t tail[2 * kBlockLen] = {};
  const size_t remaining = length - offset;
  if (remaining > 0) {
    std::memcpy(tail, data + offset, remaining);
  }
  tail[remaining] = 0x80;
  const size_t tailLen = remaining + 9 <= kBlockLen ? kBlockLen : 2 * kBlockLen;
  const uint64_t bits = static_cast<uint64_t>(length) * 8;
  for (int i = 0; i < 8; i++) {
    tail[tailLen - 1 - i] = static_cast<uint8_t>(bits >> (8 * i));
  }
  compress(state, tail);
  if (tailLen == 2 * kBlockLen) {
    compress(state, tail + kBlockLen);
  }

  for (int i = 0; i < 8; i++) {
    out[4 * i] = static_cast<uint8_t>(state[i] >> 24);
    out[4 * i + 1] = static_cast<uint8_t>(state[i] >> 16);
    out[4 * i + 2] = static_cast<uint8_t>(state[i] >> 8);
    out[4 * i + 3] = static_cast<uint8_t>(state[i]);
  }
}

void hmacSha256(const uint8_t* key, size_t keyLength, const uint8_t* data, size_t length, uint8_t out[kSha256Len]) {
  if (length > kHmacMaxMessage) {
    std::memset(out, 0, kSha256Len);
    return;
  }
  uint8_t block[kBlockLen] = {};
  if (keyLength > kBlockLen) {
    sha256(key, keyLength, block);
  } else if (keyLength > 0) {
    std::memcpy(block, key, keyLength);
  }

  uint8_t inner[kBlockLen + kHmacMaxMessage];
  for (size_t i = 0; i < kBlockLen; i++) {
    inner[i] = block[i] ^ 0x36;
  }
  if (length > 0) {
    std::memcpy(inner + kBlockLen, data, length);
  }
  uint8_t innerHash[kSha256Len];
  sha256(inner, kBlockLen + length, innerHash);

  uint8_t outer[kBlockLen + kSha256Len];
  for (size_t i = 0; i < kBlockLen; i++) {
    outer[i] = block[i] ^ 0x5C;
  }
  std::memcpy(outer + kBlockLen, innerHash, kSha256Len);
  sha256(outer, sizeof(outer), out);
}

void pbkdf2Sha256(const uint8_t* password, size_t passwordLength, const uint8_t* salt, size_t saltLength,
                  uint32_t iterations, uint8_t out[kSha256Len]) {
  std::memset(out, 0, kSha256Len);
  if (saltLength + 4 > kHmacMaxMessage || iterations == 0) {
    return;
  }
  uint8_t first[kHmacMaxMessage];
  if (saltLength > 0) {
    std::memcpy(first, salt, saltLength);
  }
  const uint8_t blockIndex[4] = {0, 0, 0, 1};
  std::memcpy(first + saltLength, blockIndex, sizeof(blockIndex));
  uint8_t u[kSha256Len];
  hmacSha256(password, passwordLength, first, saltLength + 4, u);
  std::memcpy(out, u, kSha256Len);
  for (uint32_t i = 1; i < iterations; i++) {
    uint8_t next[kSha256Len];
    hmacSha256(password, passwordLength, u, kSha256Len, next);
    std::memcpy(u, next, kSha256Len);
    for (size_t j = 0; j < kSha256Len; j++) {
      out[j] ^= u[j];
    }
  }
}

bool equalConstantTime(const uint8_t* a, const uint8_t* b, size_t length) {
  uint8_t difference = 0;
  for (size_t i = 0; i < length; i++) {
    difference |= static_cast<uint8_t>(a[i] ^ b[i]);
  }
  return difference == 0;
}

}  // namespace tadogado
