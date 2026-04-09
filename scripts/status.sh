#!/bin/bash
# status.sh - Display current project status
# Run this to orient yourself at the start of a session

set -e

echo "=== CCV Project Status ==="
echo ""

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'

# Git status
echo -e "${BLUE}=== Git Status ===${NC}"
echo "Branch: $(git branch --show-current)"
echo "Last commit: $(git log -1 --pretty=format:'%h - %s (%ar)')"
echo ""

# Check for uncommitted changes
if [ -n "$(git status --porcelain)" ]; then
    echo -e "${YELLOW}⚠ Uncommitted changes:${NC}"
    git status --short
else
    echo -e "${GREEN}✓ Working directory clean${NC}"
fi
echo ""

# Build status
echo -e "${BLUE}=== Build Status ===${NC}"
if [ -d "build" ]; then
    echo -e "${GREEN}✓ Build directory exists${NC}"
    if [ -f "build/CMakeCache.txt" ]; then
        echo "CMake cache: present"
        # Show some key CMake variables
        if [ -f "build/CMakeCache.txt" ]; then
            grep "CMAKE_BUILD_TYPE" build/CMakeCache.txt 2>/dev/null | head -1 || true
            grep "BUILD_TEST" build/CMakeCache.txt 2>/dev/null | head -1 || true
        fi
    else
        echo -e "${YELLOW}⚠ Not configured (no CMakeCache.txt)${NC}"
    fi
else
    echo -e "${YELLOW}⚠ No build directory${NC}"
    echo "Run: ./scripts/build.sh"
fi
echo ""

# Open tasks
echo -e "${BLUE}=== Open Tasks ===${NC}"
if [ -f "tasks/todo.json" ]; then
    # Count tasks by status
    HIGH_PRIORITY=$(python3 -c "import json; data=json.load(open('tasks/todo.json')); print(len([t for t in data['tasks'] if t['priority']=='high' and t['status']!='completed']))" 2>/dev/null || echo "0")
    IN_PROGRESS=$(python3 -c "import json; data=json.load(open('tasks/todo.json')); print(len([t for t in data['tasks'] if t['status']=='in_progress']))" 2>/dev/null || echo "0")
    PENDING=$(python3 -c "import json; data=json.load(open('tasks/todo.json')); print(len([t for t in data['tasks'] if t['status']=='pending']))" 2>/dev/null || echo "0")

    echo "High priority: $HIGH_PRIORITY"
    echo "In progress: $IN_PROGRESS"
    echo "Pending: $PENDING"

    # Show in-progress tasks
    if [ "$IN_PROGRESS" -gt 0 ]; then
        echo ""
        echo -e "${YELLOW}Currently in progress:${NC}"
        python3 -c "
import json
data = json.load(open('tasks/todo.json'))
for t in data['tasks']:
    if t['status'] == 'in_progress':
        print(f\"  - [{t['id']}] {t['title']}\")
" 2>/dev/null || true
    fi

    # Show high priority pending tasks
    if [ "$HIGH_PRIORITY" -gt 0 ]; then
        echo ""
        echo -e "${RED}High priority tasks:${NC}"
        python3 -c "
import json
data = json.load(open('tasks/todo.json'))
for t in data['tasks']:
    if t['priority'] == 'high' and t['status'] == 'pending':
        print(f\"  - [{t['id']}] {t['title']}\")
" 2>/dev/null || true
    fi
else
    echo -e "${YELLOW}⚠ tasks/todo.json not found${NC}"
fi
echo ""

# Test status
echo -e "${BLUE}=== Test Status ===${NC}"
if [ -d "build/unit_test" ]; then
    TEST_COUNT=$(find build/unit_test -maxdepth 1 -type f -executable | wc -l)
    echo "Test executables: $TEST_COUNT"
    find build/unit_test -maxdepth 1 -type f -executable -exec basename {} \; | sed 's/^/  - /'
else
    echo -e "${YELLOW}⚠ No test executables found${NC}"
    echo "Build with: ./scripts/build.sh"
fi
echo ""

# Documentation status
echo -e "${BLUE}=== Documentation ===${NC}"
if [ -f "docs/index.md" ]; then
    echo -e "${GREEN}✓ MkDocs documentation present${NC}"
    DOC_PAGES=$(find docs -name "*.md" | wc -l)
    echo "Documentation pages: $DOC_PAGES"
else
    echo -e "${YELLOW}⚠ No documentation found${NC}"
fi
echo ""

echo -e "${GREEN}=== Status Check Complete ===${NC}"
echo ""
echo "Next steps:"
echo "  1. Pick a task from tasks/todo.json"
echo "  2. Run ./scripts/build.sh to ensure build is current"
echo "  3. Make your changes"
echo "  4. Run ./scripts/check.sh before committing"
