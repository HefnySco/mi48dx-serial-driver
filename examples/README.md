# Examples

This folder contains example programs demonstrating the MI48 thermal camera library features.

## Build

From the repository root:

```bash
mkdir build
cd build
cmake ..
make
```

Or use the build script:

```bash
./build.sh
```

This produces these executables in `build/bin/`:

- `simple_display` - Basic thermal visualization
- `advanced_display` - Advanced processing with filters
- `raw_display` - Console-only output
- `dual_camera` - Thermal + RGB fusion
- `enhanced_display` - Interactive palette selection
- `multi_palette_display` - Auto-rotating palette display

## Common Notes

- **MI48 serial port** default is `/dev/ttyACM0`.
- If you get permission errors, add your user to `dialout` (or run with the appropriate permissions).
- Most examples start by opening the port, running a short register-init sequence, then starting the GFRA stream.

## `simple_display`

- **Purpose**
  - Basic thermal visualization using the ThermalVisualization class with RAINBOW palette.
  - Displays temperature statistics overlay and prints sampled grid to console.

- **Run**

```bash
# From project root
./build/bin/simple_display [serial_port]
```

Example:

```bash
./build/bin/simple_display /dev/ttyACM0
```

## `advanced_display`

- **Purpose**
  - Thermal visualization with optional temporal smoothing, median filtering, CLAHE, and selectable colormap.

- **Run**

```bash
# From project root
./build/bin/advanced_display [--port <serial_port>] \
  [--smooth-level <odd_int>=5] \
  [--temporal-smooth <int>=3] \
  [--clahe] \
  [--colormap <COLORMAP_RAINBOW|COLORMAP_JET|COLORMAP_HOT|COLORMAP_VIRIDIS>]
```

Example:

```bash
./build/bin/advanced_display --port /dev/ttyACM0 --smooth-level 5 --temporal-smooth 3 --colormap COLORMAP_JET
```

## `raw_display`

- **Purpose**
  - Console-only output (no OpenCV window): min/max/avg + sparse sampled grid.

- **Run**

```bash
# From project root
./build/bin/raw_display [serial_port]
```

Example:

```bash
./build/bin/raw_display /dev/ttyACM0
```

## `dual_camera`

- **Purpose**
  - Thermal + RGB fusion viewer (overlay / side-by-side / PiP) using OpenCV.
  - Supports interactive alignment (scale/offset/rotation) and saving/loading calibration.

- **Run**

```bash
# From project root
./build/bin/dual_camera [options] [thermal_port] [video_device]
```

Options:

- `--config=<file>` Load configuration from specified file
- `--help` or `-h` Show usage

Examples:

```bash
./build/bin/dual_camera
./build/bin/dual_camera /dev/ttyACM1 1
./build/bin/dual_camera --config=examples/dual_camera_calibration.ini
./build/bin/dual_camera --config=my_setup.ini /dev/ttyACM2 2
```

### Calibration files

- `dual_camera_calibration.ini`
  - A sample/working config used by the example.
- `dual_camera_calibration_example.ini`
  - Example values to start from.

### Runtime controls (in window)

Inside the UI, press `h`/`F1` to display the built-in help overlay with the full key bindings.

## `enhanced_display`

- **Purpose**
  - Interactive thermal visualization with real-time palette switching.
  - Demonstrates all available thermal color palettes.

- **Run**

```bash
# From project root
./build/bin/enhanced_display [serial_port]
```

Example:

```bash
./build/bin/enhanced_display /dev/ttyACM0
```

### Controls

- **1-9** - Select different color palettes (RAINBOW, VIVID, RGB, GRAYSCALE, MEDICAL, IRON, FIRE, OCEAN, CUSTOM)
- **H** - Toggle help display
- **Q** or **ESC** - Quit

## `multi_palette_display`

- **Purpose**
  - Automatic palette rotation display for comparing thermal visualization options.
  - Useful for evaluating which palette works best for your application.

- **Run**

```bash
# From project root
./build/bin/multi_palette_display [serial_port] [options]
```

Options:

- `--no-auto` - Disable automatic rotation, manual control only
- `--interval <frames>` - Set rotation interval (default: 30 frames)

Examples:

```bash
# Auto-rotate every 30 frames (default)
./build/bin/multi_palette_display /dev/ttyACM0

# Disable auto-rotate, manual control only
./build/bin/multi_palette_display /dev/ttyACM0 --no-auto

# Custom rotate interval (50 frames)
./build/bin/multi_palette_display /dev/ttyACM0 --interval 50
```

### Controls

- **N** - Next palette (manual)
- **P** - Previous palette (manual)
- **A** - Toggle auto-rotation on/off
- **Q** or **ESC** - Quit
