#!/bin/bash
# format.sh - Format all code according to project standards

set -e

echo "=== Formatting Code ==="

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# C++ formatting with clang-format
if command -v clang-format &> /dev/null; then
    echo "Formatting C++ files..."
    find . -type f \( \
        -name "*.cpp" -o \
        -name "*.cc" -o \
        -name "*.h" -o \
        -name "*.hpp" \
    \) ! -path "./build/*" ! -path "./.git/*" ! -path "./site/*" ! -path "./.venv/*" | \
    while read file; do
        clang-format -i "$file"
    done
    echo -e "${GREEN}✓ C++ files formatted${NC}"
else
    echo -e "${YELLOW}⚠ clang-format not found, skipping C++ formatting${NC}"
fi

# Python formatting with ruff
if command -v ruff &> /dev/null; then
    echo "Formatting Python files..."
    cd python
    ruff format .
    ruff check --fix .
    cd ..
    echo -e "${GREEN}✓ Python files formatted${NC}"
else
    echo -e "${YELLOW}⚠ ruff not found, skipping Python formatting${NC}"
fi

echo ""
echo -e "${GREEN}=== Formatting Complete ===${NC}"
