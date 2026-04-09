#!/bin/bash
# build.sh - Build CCV project with sensible defaults

set -e

echo "=== Building CCV ==="

# Default options
BUILD_TYPE="Release"
BUILD_TEST="ON"
BUILD_DOCS="OFF"
WITH_OPENCV="ON"
WITH_PCL="OFF"
WITH_GL="ON"

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --debug)
            BUILD_TYPE="Debug"
            shift
            ;;
        --no-test)
            BUILD_TEST="OFF"
            shift
            ;;
        --docs)
            BUILD_DOCS="ON"
            shift
            ;;
        --with-pcl)
            WITH_PCL="ON"
            shift
            ;;
        --clean)
            echo "Cleaning build directory..."
            rm -rf build
            mkdir -p build
            shift
            ;;
        *)
            echo "Unknown option: $1"
            echo "Usage: $0 [--debug] [--no-test] [--docs] [--with-pcl] [--clean]"
            exit 1
            ;;
    esac
done

echo "Build type: $BUILD_TYPE"
echo "Build tests: $BUILD_TEST"
echo "Build docs: $BUILD_DOCS"
echo "With OpenCV: $WITH_OPENCV"
echo "With PCL: $WITH_PCL"
echo "With GL: $WITH_GL"
echo ""

# Configure
echo "Configuring with CMake..."
cmake -S . -B build \
    -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
    -DBUILD_TEST=$BUILD_TEST \
    -DBUILD_DOCS=$BUILD_DOCS \
    -DWITH_OPENCV=$WITH_OPENCV \
    -DWITH_PCL=$WITH_PCL \
    -DWITH_GL=$WITH_GL

# Build
echo ""
echo "Building..."
cmake --build build --parallel $(($(nproc) / 2))

echo ""
echo "=== Build Complete ==="
echo "Binaries in: build/"
echo "Install with: cmake --build build --target install/strip"
