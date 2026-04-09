#!/bin/bash
# check.sh - Full validation check before commit
# Run this before committing changes

set -e

echo "=== Pre-Commit Validation ==="

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

FAILED=0

# 1. Check code formatting
echo ""
echo "=== 1. Code Formatting Check ==="
if command -v clang-format &> /dev/null; then
    # Check if any files need formatting
    FILES_TO_FORMAT=$(find . -type f \( \
        -name "*.cpp" -o \
        -name "*.cc" -o \
        -name "*.h" -o \
        -name "*.hpp" \
    \) ! -path "./build/*" ! -path "./.git/*" ! -path "./site/*" ! -path "./.venv/*" | \
    while read file; do
        if ! clang-format --dry-run --Werror "$file" 2>/dev/null; then
            echo "$file"
        fi
    done)

    if [ -z "$FILES_TO_FORMAT" ]; then
        echo -e "${GREEN}✓ All C++ files properly formatted${NC}"
    else
        echo -e "${RED}✗ Some C++ files need formatting:${NC}"
        echo "$FILES_TO_FORMAT"
        echo "Run: ./scripts/format.sh"
        FAILED=1
    fi
else
    echo -e "${YELLOW}⚠ clang-format not found, skipping format check${NC}"
fi

# 2. Build project
echo ""
echo "=== 2. Build Check ==="
if ./scripts/build.sh --no-test; then
    echo -e "${GREEN}✓ Build successful${NC}"
else
    echo -e "${RED}✗ Build failed${NC}"
    FAILED=1
fi

# 3. Run tests
echo ""
echo "=== 3. Test Check ==="
if ./scripts/test.sh; then
    echo -e "${GREEN}✓ All tests passed${NC}"
else
    echo -e "${RED}✗ Tests failed${NC}"
    FAILED=1
fi

# 4. Check for common issues
echo ""
echo "=== 4. Common Issues Check ==="

# Check for debug prints
if grep -r "std::cout << \"DEBUG" --include="*.cpp" --include="*.cc" --include="*.h" src/ include/ 2>/dev/null; then
    echo -e "${YELLOW}⚠ Found DEBUG print statements${NC}"
fi

# Check for TODO/FIXME in new code
if git diff --cached -G "TODO|FIXME" --name-only 2>/dev/null | grep -E '\.(cpp|cc|h|hpp)$'; then
    echo -e "${YELLOW}⚠ Found TODO/FIXME in staged changes${NC}"
fi

# 5. Check commit message format (if message provided)
echo ""
echo "=== 5. Commit Message Format Check ==="
echo "Commit message should follow Conventional Commits:"
echo "  <type>[scope]: <description>"
echo ""
echo "Types: feat, fix, docs, style, refactor, perf, test, chore, ci, build, revert"
echo "See: https://www.conventionalcommits.org/"
echo ""
echo "Reference: CONTRIBUTING.md"

# Summary
echo ""
echo "=== Validation Summary ==="
if [ $FAILED -eq 0 ]; then
    echo -e "${GREEN}✓ All checks passed! Ready to commit.${NC}"
    exit 0
else
    echo -e "${RED}✗ Some checks failed. Please fix before committing.${NC}"
    exit 1
fi
