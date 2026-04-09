#!/bin/bash
# test.sh - Run all tests for CCV

set -e

echo "=== Running CCV Tests ==="

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Check if build exists
if [ ! -d "build" ]; then
    echo -e "${YELLOW}Build directory not found. Running build first...${NC}"
    ./scripts/build.sh
fi

# Run CTest
echo ""
echo "=== CTest ==="
cd build
if ctest --output-on-failure; then
    echo -e "${GREEN}✓ All CTest tests passed${NC}"
else
    echo -e "${RED}✗ Some CTest tests failed${NC}"
    exit 1
fi
cd ..

# Run individual test executables if they exist
echo ""
echo "=== Individual Test Suites ==="

run_test_suite() {
    local test_name=$1
    local test_exe="build/unit_test/$test_name"

    if [ -f "$test_exe" ]; then
        echo "Running $test_name..."
        # Run from unit_test directory so test data paths resolve correctly
        cd build/unit_test
        if ./$test_name; then
            echo -e "${GREEN}✓ $test_name passed${NC}"
        else
            echo -e "${RED}✗ $test_name failed${NC}"
            cd ../..
            return 1
        fi
        cd ../..
    else
        echo -e "${YELLOW}⚠ $test_name not found (skipping)${NC}"
    fi
}

run_test_suite "test_maths"
run_test_suite "test_cv"
run_test_suite "test_kinematics"

# Python tests
echo ""
echo "=== Python Tests ==="
cd python
if [ -d "tests" ] || [ -d "test" ]; then
    if python3 -m pytest --version &> /dev/null; then
        if python3 -m pytest -v; then
            echo -e "${GREEN}✓ Python tests passed${NC}"
        else
            echo -e "${YELLOW}⚠ Some Python tests failed${NC}"
        fi
    else
        echo -e "${YELLOW}⚠ pytest not installed, skipping Python tests${NC}"
    fi
else
    echo "No Python tests found"
fi
cd ..

echo ""
echo -e "${GREEN}=== All Tests Complete ===${NC}"
