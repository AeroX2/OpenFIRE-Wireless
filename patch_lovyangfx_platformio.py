#!/usr/bin/env python3
"""
PlatformIO script to patch LovyanGFX Bus_RGB.cpp file using patch.py
Replaces deprecated gpio_hal_iomux_func_sel with gpio_iomux_out
"""

import os
import glob
from os.path import join, isfile

Import("env")

def apply_lovyangfx_patch():
    """
    Apply the LovyanGFX Bus_RGB.cpp patch
    """
    print("Looking for LovyanGFX Bus_RGB.cpp file to patch...")
    
    # Find the file to patch
    file_path = find_bus_rgb_file()
    
    if not file_path:
        print("Error: Bus_RGB.cpp file not found in any expected location.")
        print("Make sure you have built the dongle environment first.")
        return False
    
    print(f"Found file to patch: {file_path}")
    
    # Create patch flag file path
    patchflag_path = file_path + ".patched"
    
    # Check if already patched
    if isfile(patchflag_path):
        print("File already patched, skipping...")
        return True
    
    try:
        # Read the file to patch
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # Define the old and new patterns
        old_pattern = 'gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[pin], PIN_FUNC_GPIO);'
        new_pattern = 'gpio_iomux_out(pin, PIN_FUNC_GPIO, false);'
        
        # Check if the old pattern exists
        if old_pattern not in content:
            print("Warning: The pattern to replace was not found in the file.")
            print("The file may have already been patched or the pattern has changed.")
            return False
        
        # Perform the replacement
        new_content = content.replace(old_pattern, new_pattern)
        
        # Check if any changes were made
        if new_content == content:
            print("No changes were made. The pattern may not match exactly.")
            return False
        
        # Write the patched content back to the file
        with open(file_path, 'w', encoding='utf-8') as f:
            f.write(new_content)
        
        print(f"Successfully patched {file_path}")
        print(f"Replaced: {old_pattern}")
        print(f"With: {new_pattern}")
        
        # Create patch flag file
        with open(patchflag_path, 'w') as f:
            f.write("patched")
        
        return True
        
    except Exception as e:
        print(f"Error applying patch: {e}")
        return False

def find_bus_rgb_file():
    """
    Find the Bus_RGB.cpp file in the project
    """
    # Common paths where the file might be located
    possible_paths = [
        ".pio/libdeps/dongle/LovyanGFX/src/lgfx/v1/platforms/esp32s3/Bus_RGB.cpp",
        ".pio/libdeps/dongle/lovyan03/LovyanGFX/src/lgfx/v1/platforms/esp32s3/Bus_RGB.cpp",
        ".pio/libdeps/dongle/LovyanGFX/lgfx/v1/platforms/esp32s3/Bus_RGB.cpp",
        ".pio/libdeps/dongle/lovyan03/LovyanGFX/lgfx/v1/platforms/esp32s3/Bus_RGB.cpp"
    ]
    
    # Check specific paths first
    for path in possible_paths:
        if isfile(path):
            return path
    
    # Search using glob patterns
    search_patterns = [
        ".pio/libdeps/**/LovyanGFX/**/Bus_RGB.cpp",
        ".pio/libdeps/**/lovyan03/**/Bus_RGB.cpp"
    ]
    
    for pattern in search_patterns:
        matches = glob.glob(pattern, recursive=True)
        if matches:
            return matches[0]
    
    return None

# Run the patch for dongle environment
env_name = env.GetProjectOption("default_envs", "")
if "dongle" in env_name or env.GetProjectOption("board", "") == "esp32-s3-devkitc-1":
    print("Dongle environment detected, running LovyanGFX patch...")
    apply_lovyangfx_patch()
else:
    print("Not dongle environment, skipping LovyanGFX patch...")
