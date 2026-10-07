#!/usr/bin/env bash
# Fetch the pinned zenoh-pico for the experimental CONFIG_RAFTROS_BACKEND_ZENOH_PICO build.
#
# zenoh-pico (Eclipse, EPL-2.0 OR Apache-2.0) is a third-party dependency and is
# not part of this repository: it is cloned into third_party/zenoh-pico, which is
# git-ignored.  The default build (Raft's own Zenoh implementation) and the RTPS
# build never use it.  A firmware image built with it must carry zenoh-pico's
# licence and NOTICE (third_party/zenoh-pico/LICENSE, NOTICE.md).
#
# Usage: tools/fetch_zenoh_pico.sh            (re-running is safe)
set -euo pipefail

ZENOH_PICO_TAG="1.10.1"
ZENOH_PICO_COMMIT="e1ab223a28aaebb5dec1e70d98eab152332f777a"
ZENOH_PICO_URL="https://github.com/eclipse-zenoh/zenoh-pico.git"

cd "$(dirname "$0")/.."
dest="third_party/zenoh-pico"
if [ ! -d "$dest/.git" ]; then
    mkdir -p third_party
    git clone --quiet "$ZENOH_PICO_URL" "$dest"
fi
git -C "$dest" fetch --quiet --tags origin
git -C "$dest" -c advice.detachedHead=false checkout --quiet "$ZENOH_PICO_TAG"
actual="$(git -C "$dest" rev-parse HEAD)"
if [ "$actual" != "$ZENOH_PICO_COMMIT" ]; then
    echo "zenoh-pico $ZENOH_PICO_TAG is $actual, expected $ZENOH_PICO_COMMIT - refusing to build against it" >&2
    exit 1
fi
echo "zenoh-pico $ZENOH_PICO_TAG ($actual) in $dest"
