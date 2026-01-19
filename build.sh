#!/bin/bash
# Build script for mi48 library
# Usage: ./build.sh [DEBUG|RELEASE]

BUILD_TYPE=${1:-DEBUG}

echo "Building mi48 library in $BUILD_TYPE mode..."

rm -rf ./build
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=$BUILD_TYPE -D CMAKE_INSTALL_PREFIX=$HOME/.local ../
make

echo ""
echo "Build complete!"
echo "To install the library, run: ./install.sh"

