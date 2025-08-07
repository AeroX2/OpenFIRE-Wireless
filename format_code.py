#!/usr/bin/env python3
"""
OpenFIRE Code Formatter Script
Automatically formats all C/C++ files in the project using clang-format.
"""

import os
import subprocess
import sys
from pathlib import Path

def find_cpp_files(directory):
    """Find all C/C++ files in the given directory and subdirectories."""
    cpp_extensions = {'.cpp', '.c', '.h', '.hpp', '.cc', '.cxx', '.hxx'}
    cpp_files = []
    
    for root, dirs, files in os.walk(directory):
        # Skip .git and .pio directories
        dirs[:] = [d for d in dirs if d not in {'.git', '.pio'}]
        
        for file in files:
            if Path(file).suffix in cpp_extensions:
                cpp_files.append(os.path.join(root, file))
    
    return cpp_files

def format_file(file_path):
    """Format a single file using clang-format."""
    try:
        result = subprocess.run(
            ['clang-format', '-i', file_path],
            capture_output=True,
            text=True,
            check=True
        )
        print(f"✓ Formatted: {file_path}")
        return True
    except subprocess.CalledProcessError as e:
        print(f"✗ Error formatting {file_path}: {e}")
        return False
    except FileNotFoundError:
        print("✗ clang-format not found. Please install clang-format.")
        return False

def main():
    """Main function to format all C/C++ files in the project."""
    project_root = Path(__file__).parent
    cpp_files = find_cpp_files(project_root)
    
    if not cpp_files:
        print("No C/C++ files found to format.")
        return
    
    print(f"Found {len(cpp_files)} C/C++ files to format...")
    
    success_count = 0
    for file_path in cpp_files:
        if format_file(file_path):
            success_count += 1
    
    print(f"\nFormatting complete: {success_count}/{len(cpp_files)} files formatted successfully.")
    
    if success_count < len(cpp_files):
        sys.exit(1)

if __name__ == "__main__":
    main() 