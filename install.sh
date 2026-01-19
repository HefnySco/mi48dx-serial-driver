#!/bin/bash
# Install script for mi48 library
# This installs the library to $HOME/.local

if [ ! -d "build" ]; then
    echo "Error: build directory not found. Please run ./build.sh first."
    exit 1
fi

echo "Installing mi48 library to $HOME/.local..."

cd build
make install

echo ""
echo "Installation complete!"
echo "Library installed to: $HOME/.local/lib"
echo "Headers installed to: $HOME/.local/include"
echo "CMake config installed to: $HOME/.local/lib/cmake/mi48"
