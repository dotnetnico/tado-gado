#!/usr/bin/env bash
# Prints "true" when the commits from <base> to HEAD change more than documentation,
# else "false". Documentation is every Markdown file, docs/ and LICENSE.
#
# Usage: firmware_changed.sh <base>
#
# An unknown base, or no changed files at all, prints "true": when in doubt, CI builds
# and tests everything.
set -euo pipefail

base=${1:-}
if [ -z "$base" ] || ! git cat-file -e "${base}^{commit}" 2>/dev/null; then
  echo true
  exit 0
fi

changed=$(git diff --name-only "$base" HEAD)
if [ -z "$changed" ]; then
  echo true
  exit 0
fi

while IFS= read -r path; do
  case "$path" in
    *.md | docs/* | LICENSE) ;;
    *)
      echo true
      exit 0
      ;;
  esac
done <<<"$changed"
echo false
