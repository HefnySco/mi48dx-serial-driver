#!/bin/bash
# Uninstall script for mi48 library
# This removes the library from $HOME/.local

INSTALL_PREFIX=$HOME/.local

echo "Uninstalling mi48 library from $INSTALL_PREFIX..."

# Remove library files
rm -f $INSTALL_PREFIX/lib/libmi48.a
rm -f $INSTALL_PREFIX/lib/libmi48.so

# Remove header files
rm -f $INSTALL_PREFIX/include/serial_mi48.hpp
rm -f $INSTALL_PREFIX/include/thermal_visualization.hpp
rm -f $INSTALL_PREFIX/include/version.hpp

# Remove CMake config files
rm -rf $INSTALL_PREFIX/lib/cmake/mi48

echo ""
echo "Uninstallation complete!"
echo "mi48 library has been removed from $INSTALL_PREFIX"
