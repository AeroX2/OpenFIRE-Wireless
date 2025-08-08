# LovyanGFX Bus_RGB.cpp Patch

This patch fixes a compilation error in the LovyanGFX library for ESP32-S3 boards. The error occurs because the `gpio_hal_iomux_func_sel` function has been deprecated and replaced with `gpio_iomux_out`.

## Files Included

- `lovyangfx_bus_rgb.patch` - The patch file that contains the fix
- `apply_lovyangfx_patch.py` - Python script to apply the patch manually
- `patch_lovyangfx_platformio.py` - PlatformIO script for automatic patching during build
- `patch_dongle.bat` - Windows batch script for easy manual patching

## How to Use

### Option 1: Manual Patching (Recommended)

1. Make sure you have Python installed
2. Make sure `patch.py` is in the same directory as the scripts
3. Run the patch script:
   ```bash
   python apply_lovyangfx_patch.py
   ```
   Or on Windows:
   ```cmd
   patch_dongle.bat
   ```

### Option 2: Automatic Patching with PlatformIO

The patch is configured to run automatically when building the dongle environment. The `platformio.ini` file includes:

```ini
extra_scripts = pre:patch_lovyangfx_platformio.py
```

This will automatically apply the patch before building.

## What the Patch Does

The patch replaces:
```cpp
gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[pin], PIN_FUNC_GPIO);
```

With:
```cpp
gpio_iomux_out(pin, PIN_FUNC_GPIO, false);
```

## Troubleshooting

1. **"patch.py module not found"** - Make sure `patch.py` is in the same directory
2. **"Bus_RGB.cpp file not found"** - Build the dongle environment first to download the library
3. **"Patch file not found"** - Make sure `lovyangfx_bus_rgb.patch` is in the same directory

## Verification

After applying the patch, you should be able to build the dongle environment without the GPIO error:

```bash
pio run -e dongle
```

The patch creates a `.patched` flag file to prevent re-applying the patch multiple times.
