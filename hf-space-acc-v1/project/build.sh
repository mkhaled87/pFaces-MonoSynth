#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# Note: You need to first install OpenSSL and CppRestSDK before building the kernel (for MacOS):
#   $ brew install openssl
# You may also need to export the location of the OpenSSL library:
#   $ export OPENSSL_ROOT_DIR=/usr/local/opt/openssl

# BUILD_TYPE = Release | Debug
BUILD_TYPE=Release

# Clean build ?
CLEAN_BUILD=false

# remove old build binaries
if [ "$CLEAN_BUILD" = "true" ]; then
    rm -rf "$SCRIPT_DIR/kernel-pack/"*.driver
    rm -rf "$SCRIPT_DIR/build"
fi

# building ...
mkdir -p "$SCRIPT_DIR/build"
cd "$SCRIPT_DIR/build"
cmake .. -DCMAKE_BUILD_TYPE="$BUILD_TYPE"
cmake --build . --config "$BUILD_TYPE"
