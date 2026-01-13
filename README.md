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

### Using Make (Recommended for quick builds)
```bash
# Build libraries and examples
make all

# Build only libraries
make libraries

# Install libraries system-wide (requires sudo)
sudo make install

# Clean build files
make clean

# Show help
make help
```

### Using CMake (Recommended for integration)
```bash
mkdir build
cd build
cmake ..
make

# Install libraries system-wide
sudo make install
```

## Library Files Generated
- `libmi48.a` - Static library
- `libmi48.so` - Shared library
- `serial_mi48.hpp` - Header file

## Usage in Your Projects

### Method 1: Using pkg-config
```bash
# Compile your application
g++ -std=c++17 your_app.cpp `pkg-config --cflags --libs mi48` -o your_app
```

### Method 2: Manual linking with Make
```makefile
CXX = g++
CXXFLAGS = -std=c++17 -I/path/to/mi48_lib_c
LDFLAGS = -L/path/to/mi48_lib_c -lmi48 -lserialport `pkg-config --libs opencv4`

your_app: your_app.o
	$(CXX) your_app.o -o $@ $(LDFLAGS)
```

### Method 3: Using CMake
```cmake
find_package(PkgConfig REQUIRED)
pkg_check_modules(MI48 REQUIRED mi48)

add_executable(your_app your_app.cpp)
target_include_directories(your_app PRIVATE ${MI48_INCLUDE_DIRS})
target_link_libraries(your_app PRIVATE ${MI48_LIBRARIES})
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
   make all
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
