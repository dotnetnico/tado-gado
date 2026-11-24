// What this build is. scripts/build_info.py sets both values.
#pragma once

#ifndef TADO_GADO_VERSION
#define TADO_GADO_VERSION "0.0.0"
#endif
#ifndef TADO_GADO_VARIANT
#define TADO_GADO_VARIANT "unknown"
#endif

constexpr char kVariant[] = TADO_GADO_VARIANT;

// An update is recognised by this string in its image, see image_check.h. The version
// is read from it, so the linker cannot leave it out.
extern const char kImageMarker[];
inline const char* firmwareVersion() { return kImageMarker + sizeof("tado-gado-image:" TADO_GADO_VARIANT ":") - 1; }
