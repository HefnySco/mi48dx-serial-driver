# MI48 Thermal Camera Library

## Overview
This project provides a C++ library for interfacing with the MI48 thermal imaging sensor, processing thermal data, and visualizing it. The library can be built as both static and shared libraries for easy integration into other applications.

This is a C++ alternative to Python [MeridianInnovation PySenxor](https://github.com/HefnySco/pysenxor-lite)

![Sample](https://github.com/HefnySco/mi48dx-serial-driver/blob/master/res/img1.png?raw=true)

## Requirements
- **Hardware**:
  - MI48 thermal imaging sensor
  - Optional: USB webcam for visual input
- **Software**:
  - C++ compiler (g++ with C++17 support)
  - OpenCV 4.x
  - libserialport
  - POSIX-compliant system for serial port operations

## Building the Library

### Quick Start (Recommended)
```bash
# Build the library (creates DEBUG build by default)
./build.sh

# Or build in RELEASE mode
./build.sh RELEASE

# Install to $HOME/.local
./install.sh

# Uninstall if needed
./uninstall.sh
```

### Manual Build Using CMake
```bash
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=DEBUG -DCMAKE_INSTALL_PREFIX=$HOME/.local ..
make
make install
```

## Library Files Generated

After building, you'll find in `build/bin/`:
- `libmi48.a` - Static library
- `libmi48.so` - Shared library

After installation (default: `$HOME/.local`):
- **Libraries**: `$HOME/.local/lib/libmi48.{a,so}`
- **Headers**: `$HOME/.local/include/{serial_mi48.hpp,thermal_visualization.hpp,version.hpp}`
- **CMake config**: `$HOME/.local/lib/cmake/mi48/`

## Using the Library in Your Projects

### Method 1: Using CMake find_package (Recommended)
```cmake
# Find the installed mi48 library
find_package(mi48 REQUIRED PATHS $ENV{HOME}/.local/lib/cmake/mi48)

add_executable(your_app your_app.cpp)

# Link against the static library
target_link_libraries(your_app PRIVATE mi48::mi48_static serialport)

# Or link against the shared library
# target_link_libraries(your_app PRIVATE mi48::mi48_shared serialport)
```

### Method 2: Manual Linking
```bash
# Compile with static library
g++ -std=c++17 your_app.cpp \
    -I$HOME/.local/include \
    -L$HOME/.local/lib \
    -lmi48 -lserialport \
    `pkg-config --cflags --libs opencv4` \
    -o your_app
```

## Basic Usage Example
```cpp
#include "serial_mi48.hpp"
#include <iostream>

int main() {
    SerialCommandSender camera;
    
    // Register callback for frame data
    camera.register_frame_callback([](const std::vector<float>& temps, uint16_t rows, uint16_t cols) {
        std::cout << "Received frame: " << rows << "x" << cols << std::endl;
        // Process temperature data here
    });
    
    // Open connection
    if (!camera.open_port("/dev/ttyACM0")) {
        std::cerr << "Failed to open port" << std::endl;
        return 1;
    }
    
    // Start streaming
    if (camera.start_stream(true)) {
        std::cout << "Streaming started..." << std::endl;
        camera.loop_on_read(); // Blocks until stop_loop() is called
    }
    
    return 0;
}
```

## Installation

### Quick Installation
1. Clone the repository:
   ```bash
   git clone https://github.com/HefnySco/mi48dx-serial-driver.git
   cd mi48dx-serial-driver
   ```

2. Build and install:
   ```bash
   ./build.sh          # Build the library
   ./install.sh        # Install to $HOME/.local
   ```

### Custom Installation Prefix
If you want to install to a different location:
```bash
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/your/custom/path ..
make
make install
```

### Uninstallation
To remove the installed library:
```bash
./uninstall.sh
```

## Examples

See [`examples/README.md`](examples/README.md) for detailed example programs.

## API Reference

The library provides the following main classes and functions:

- `SerialCommandSender` - Main class for camera communication
- Frame callback mechanism for real-time data processing
- Register read/write functions for camera configuration
- Streaming control functions

## Library Version
- Version: 2.0.0
- Use `SerialCommandSender::get_version()` to get runtime version info.

## Notes
- The library installs to `$HOME/.local` by default (no sudo required)
- Make sure `$HOME/.local/lib` is in your library path
- CMake will automatically find the library if installed to `$HOME/.local`
- Dependencies: OpenCV 4.x and libserialport must be installed on your system
