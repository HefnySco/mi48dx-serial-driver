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

### Using CMake
```bash
mkdir build
cd build
cmake ..
make

# Install libraries system-wide (requires sudo)
sudo make install
```

### Using the Build Script (Quick Start)
```bash
./build.sh
```

This creates a debug build in the `build/` directory with all libraries and examples.

## Library Files Generated
After building, you'll find in `build/bin/`:
- `libmi48.a` - Static library
- `libmi48.so` - Shared library

Headers are in the project root:
- `serial_mi48.hpp` - Main camera interface
- `thermal_visualization.hpp` - Visualization utilities
- `version.hpp` - Version information

## Usage in Your Projects

### Method 1: Using CMake (Recommended)
```cmake
find_package(PkgConfig REQUIRED)
pkg_check_modules(MI48 REQUIRED mi48)

add_executable(your_app your_app.cpp)
target_include_directories(your_app PRIVATE ${MI48_INCLUDE_DIRS})
target_link_libraries(your_app PRIVATE ${MI48_LIBRARIES})
```

### Method 2: Using pkg-config (After Installation)
```bash
# Compile your application
g++ -std=c++17 your_app.cpp `pkg-config --cflags --libs mi48` -o your_app
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
1. Clone the repository:
   ```bash
   git clone https://github.com/HefnySco/mi48dx-serial-driver.git
   cd mi48dx-serial-driver/mi48_lib_c
   ```
2. Build and install:
   ```bash
   mkdir build
   cd build
   cmake ..
   make
   sudo make install
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
- Version: 1.0.0
- Use `SerialCommandSender::get_version()` to get runtime version info.
