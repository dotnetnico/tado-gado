#!/usr/bin/env bash
# Static analysis of the C++ code. Run from the repository root.
#
# Without the Arduino and FreeRTOS headers cppcheck cannot resolve those includes,
# so it reports only what it can tell from the code itself.
set -euo pipefail

cppcheck \
  --enable=warning,performance,portability \
  --language=c++ --std=c++17 \
  --inline-suppr \
  --suppress=missingInclude \
  --suppress=missingIncludeSystem \
  --error-exitcode=1 \
  --quiet \
  -I firmware/lib/tado_gado_core/src \
  firmware/lib/tado_gado_core/src firmware/src
