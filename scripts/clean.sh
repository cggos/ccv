#!/bin/bash
# clean.sh - Clean build artifacts

set -e

echo "=== Cleaning Build Artifacts ==="

# Remove build directory
if [ -d "build" ]; then
    echo "Removing build directory..."
    rm -rf build
    echo "✓ build/ removed"
fi

# Remove Python build artifacts
cd python
if [ -d "build" ]; then
    rm -rf build
    echo "✓ python/build/ removed"
fi
if [ -d "dist" ]; then
    rm -rf dist
    echo "✓ python/dist/ removed"
fi
if [ -d "*.egg-info" ]; then
    rm -rf *.egg-info
    echo "✓ egg-info removed"
fi
cd ..

# Clean CMake cache files
find . -name "CMakeCache.txt" -delete 2>/dev/null || true
find . -name "CMakeFiles" -type d -exec rm -rf {} + 2>/dev/null || true

echo ""
echo "=== Clean Complete ==="
echo "Run ./scripts/init.sh to re-initialize"
