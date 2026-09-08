#!/bin/sh

# Note: You need to first install OpenSSL and CppRestSDK before building the kernel (for MacOS):
#   $ brew install openssl
# You may also need to export the location of the OpenSSL library:
#   $ export OPENSSL_ROOT_DIR=/usr/local/opt/openssl

# BUILD_TYPE = Release | Debug 
BUILD_TYPE=Release

# Clean build ?
CLEAN_BUILD=false

# remove old build binaries
if [ $CLEAN_BUILD = true ]
then 
    rm -rf kernel-pack/*.driver
    rm -rf build
fi

# building ...
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DBUILD_TESTING=ON
cmake --build . --config $BUILD_TYPE
cd ..