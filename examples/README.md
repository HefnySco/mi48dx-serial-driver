# Examples

This folder contains small example programs built from the top-level `Makefile`.

## Build

From the repository root:

```bash
make all
```

This produces these executables in the repository root:

- `simple_display`
- `advanced_display`
- `raw_display`
- `dual_camera`

## Common Notes

- **MI48 serial port** default is `/dev/ttyACM0`.
- If you get permission errors, add your user to `dialout` (or run with the appropriate permissions).
- Most examples start by opening the port, running a short register-init sequence, then starting the GFRA stream.

## `simple_display`

- **Purpose**
  - Basic thermal visualization using OpenCV (colormap + resize + rotate).
  - Prints basic frame stats to console.

- **Run**

```bash
./simple_display [serial_port]
```

Example:

```bash
./simple_display /dev/ttyACM0
```

## `advanced_display`

- **Purpose**
  - Thermal visualization with optional temporal smoothing, median filtering, CLAHE, and selectable colormap.

- **Run**

```bash
./advanced_display [--port <serial_port>] \
  [--smooth-level <odd_int>=5] \
  [--temporal-smooth <int>=3] \
  [--clahe] \
  [--colormap <COLORMAP_RAINBOW|COLORMAP_JET|COLORMAP_HOT|COLORMAP_VIRIDIS>]
```

Example:

```bash
./advanced_display --port /dev/ttyACM0 --smooth-level 5 --temporal-smooth 3 --colormap COLORMAP_JET
```

## `raw_display`

- **Purpose**
  - Console-only output (no OpenCV window): min/max/avg + sparse sampled grid.

- **Run**

```bash
./raw_display [serial_port]
```

Example:

```bash
./raw_display /dev/ttyACM0
```

## `dual_camera`

- **Purpose**
  - Thermal + RGB fusion viewer (overlay / side-by-side / PiP) using OpenCV.
  - Supports interactive alignment (scale/offset/rotation) and saving/loading calibration.

- **Run**

```bash
./dual_camera [options] [thermal_port] [video_device]
```

Options:

- `--config=<file>` Load configuration from specified file
- `--help` or `-h` Show usage

Examples:

```bash
./dual_camera
./dual_camera /dev/ttyACM1 1
./dual_camera --config=dual_camera_calibration.ini
./dual_camera --config=my_setup.ini /dev/ttyACM2 2
```

### Calibration files

- `dual_camera_calibration.ini`
  - A sample/working config used by the example.
- `dual_camera_calibration_example.ini`
  - Example values to start from.

### Runtime controls (in window)

Inside the UI, press `h`/`F1` to display the built-in help overlay with the full key bindings.
