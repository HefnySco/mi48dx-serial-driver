# Thermal Visualization for MI48 Camera

This document describes the enhanced thermal visualization capabilities added to your MI48 thermal camera library.

## Overview

The new thermal visualization system provides multiple color palettes for displaying temperature data from your MI48 camera, replacing the basic rainbow colormap with a comprehensive set of visualization options.

## Features

### Available Color Palettes

1. **RAINBOW** - Original OpenCV rainbow colormap
2. **VIVID** - High-contrast vivid colors (Blue → Cyan → Green → Yellow → Red → Magenta)
3. **RGB** - RGB-based thermal mapping (Blue → Cyan → Green → Yellow → White)
4. **GRAYSCALE** - Simple grayscale intensity mapping
5. **MEDICAL** - Medical thermal palette (Black → Blue → Green → Yellow → Red → White)
6. **IRON** - Iron/black hot palette (Black → Dark red → Red → Orange → Yellow → White)
7. **FIRE** - Fire palette (Black → Red → Yellow → White)
8. **OCEAN** - Ocean/blue palette (Black → Deep blue → Blue → Cyan → White)
9. **CUSTOM** - User-defined color palette

### Key Classes

#### `ThermalVisualization`
Main visualization class that handles:
- Temperature data normalization
- Color palette application
- Image scaling and rotation
- Custom palette support

#### Example Programs

1. **`enhanced_display.cpp`** - Interactive palette selection
   - Press keys 1-9 to select different palettes
   - Toggle help display with 'H'
   - Quit with 'Q' or ESC

2. **`multi_palette_display.cpp`** - Auto-rotating palette display
   - Automatically cycles through all palettes
   - Manual control with N/P keys
   - Toggle auto-rotate with 'A'
   - Command-line options for customization

3. **`simple_display.cpp`** - Updated to use RAINBOW palette
   - Simplified interface using the new visualization class
   - Displays temperature statistics and sampled grid

## Usage

### Basic Usage

```cpp
#include "thermal_visualization.hpp"

ThermalVisualization visualizer;
cv::Mat thermal_image = visualizer.visualizeTemperatures(
    temperatures, rows, cols, 
    ThermalPalette::VIVID,  // or any other palette
    -273.15f, 100.0f,      // min/max temperature range
    4,                      // scale factor
    true                    // rotate image
);
```

### Custom Palette

```cpp
std::vector<cv::Vec3b> custom_colors = {
    cv::Vec3b(0, 0, 0),      // Black
    cv::Vec3b(0, 0, 255),    // Blue
    cv::Vec3b(0, 255, 0),    // Green
    cv::Vec3b(255, 0, 0),    // Red
    cv::Vec3b(255, 255, 255)  // White
};

visualizer.setCustomPalette(custom_colors);
cv::Mat image = visualizer.visualizeTemperatures(
    temperatures, rows, cols, ThermalPalette::CUSTOM
);
```

### Palette Selection by Name

```cpp
std::string palette_name = "VIVID";
ThermalPalette palette = ThermalVisualization::paletteFromString(palette_name);
```

## Building

The new visualization requires OpenCV. The project uses CMake as the build system:

```bash
mkdir build
cd build
cmake ..
make
```

Or use the provided build script:

```bash
./build.sh
```

For integration in your own projects:

```cmake
find_package(PkgConfig REQUIRED)
pkg_check_modules(OPENCV REQUIRED opencv4)
target_link_libraries(your_target ${OPENCV_LIBRARIES})
```

## Examples

### Build All Examples

```bash
# From project root
mkdir build
cd build
cmake ..
make

# Or use the build script
./build.sh
```

### Run Enhanced Display

```bash
# From project root after building
./build/bin/enhanced_display /dev/ttyACM0
```

### Run Multi-Palette Display

```bash
# Auto-rotate every 30 frames (default)
./build/bin/multi_palette_display /dev/ttyACM0

# Disable auto-rotate, manual control only
./build/bin/multi_palette_display /dev/ttyACM0 --no-auto

# Custom rotate interval
./build/bin/multi_palette_display /dev/ttyACM0 --interval 50
```

## Palette Descriptions

### VIVID
High-contrast palette optimized for thermal imaging with clear distinction between temperature ranges. Ideal for general-purpose thermal visualization.

### RGB
Smooth gradient from cold (blue) to hot (white) through intermediate colors. Provides natural temperature representation.

### MEDICAL
Medical imaging palette with smooth transitions optimized for biological temperature ranges. Commonly used in medical thermography.

### IRON
Traditional "iron" or "black hot" palette used in industrial and military thermal imaging. Black represents cold, white represents hot.

### FIRE
Fire-themed palette emphasizing hot zones with red, orange, and yellow colors. Excellent for highlighting heat sources.

### OCEAN
Cool-toned palette with blues and cyans. Good for applications where cooler temperatures are of primary interest.

## Integration Notes

The new visualization system is designed to be a drop-in replacement for the existing display code while maintaining compatibility with your current MI48 camera interface.

Key improvements:
- Simplified API - single function call for complete visualization
- Better performance - optimized color mapping
- Flexible configuration - easy palette switching
- Professional appearance - proper scaling and text overlay

The system automatically handles temperature normalization, image scaling (4x by default), and rotation correction for your MI48 camera.
