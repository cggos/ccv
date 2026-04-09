#!/bin/bash
# init.sh - Initialize development environment for CCV
# Run this at the start of each development session

set -e

echo "=== CCV Development Environment Initialization ==="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check CMake version
CMAKE_VERSION=$(cmake --version | head -n1 | grep -oP '\d+\.\d+\.\d+' || echo "0.0.0")
CMAKE_MAJOR=$(echo $CMAKE_VERSION | cut -d. -f1)
if [ "$CMAKE_MAJOR" -lt 3 ]; then
    echo -e "${RED}ERROR: CMake 3.20+ required, found $CMAKE_VERSION${NC}"
    exit 1
fi
echo -e "${GREEN}✓ CMake version: $CMAKE_VERSION${NC}"

# Check C++ compiler version
echo ""
echo "=== Compiler Version Check ==="
CXX_VERSION=$(${CMAKE_CXX_COMPILER:-g++} --version | head -n1)
echo "C++ Compiler: $CXX_VERSION"
echo -e "${GREEN}✓ C++17 support required${NC}"

# Check for required tools
check_tool() {
    if command -v "$1" &> /dev/null; then
        echo -e "${GREEN}✓ $1 found${NC}"
        return 0
    else
        echo -e "${YELLOW}⚠ $1 not found (optional)${NC}"
        return 1
    fi
}

check_tool clang-format
check_tool doxygen
check_tool python3
check_tool pip

# Check for GTest (optional but recommended)
if pkg-config --exists gtest; then
    echo -e "${GREEN}✓ GTest found${NC}"
else
    echo -e "${YELLOW}⚠ GTest not found (install for unit tests)${NC}"
fi

# Create build directory if not exists
if [ ! -d "build" ]; then
    echo "Creating build directory..."
    mkdir -p build
fi

# Check Python environment
echo ""
echo "=== Python Environment ==="
PYTHON_VERSION=$(python3 --version 2>/dev/null | grep -oP '\d+\.\d+' || echo "0.0")
PYTHON_MAJOR=$(echo $PYTHON_VERSION | cut -d. -f1)
PYTHON_MINOR=$(echo $PYTHON_VERSION | cut -d. -f2)
if [ "$PYTHON_MAJOR" -ge 3 ] && [ "$PYTHON_MINOR" -ge 10 ]; then
    echo -e "${GREEN}✓ Python version: $PYTHON_VERSION (3.10+ required)${NC}"
else
    echo -e "${RED}ERROR: Python 3.10+ required, found $PYTHON_VERSION${NC}"
    exit 1
fi

cd python
if [ -f "pyproject.toml" ]; then
    echo "Python package config found"
    # Check if libccv is installed
    if pip show libccv &> /dev/null; then
        echo -e "${GREEN}✓ libccv Python package installed${NC}"
    else
        echo -e "${YELLOW}⚠ libccv not installed. Run: cd python && pip install .${NC}"
    fi
fi
cd ..

# Verify Eigen3
if [ -d "$CG_APP_RELEASE" ]; then
    echo -e "${GREEN}✓ CG_APP_RELEASE set: $CG_APP_RELEASE${NC}"
else
    echo -e "${YELLOW}⚠ CG_APP_RELEASE not set. Eigen3 may not be found.${NC}"
fi

echo ""
echo "=== Configuration ==="
echo "Build directory: $(pwd)/build"
echo "Install prefix: /opt/ccv"
echo ""
echo "Ready for development! Next steps:"
echo "  1. Run ./scripts/build.sh to build the project"
echo "  2. Run ./scripts/test.sh to verify tests pass"
echo "  3. Check tasks/todo.json for open tasks"
