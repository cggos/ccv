# AGENTS.md

This file provides the entry point for AI agents (Claude Code, etc.) working on the CCV repository.

**Quick Start**: Run `./scripts/init.sh` to set up the development environment.

---

## Project Overview

CCV is a C++ computer vision library with zero third-party dependencies in core logic.

**Requirements:**
- **C++17 or higher** (set in CMakeLists.txt)
- **Python 3.10 or higher** (for Python bindings)

**Key Principles:**
- Core algorithms have **no dependencies** (no OpenCV/Eigen/PCL in core)
- Uses `FLOAT` macro for single/double precision toggle
- All code in `cg` namespace
- Google C++ Style, 120 column limit

## Session Protocol

1. **Orient**: Read `CLAUDE.md`, check `tasks/todo.json` for open tasks
2. **Setup**: Run `./scripts/init.sh` to verify environment
3. **Verify Baseline**: Run `./scripts/test.sh` to ensure tests pass
4. **Select One Task**: Pick highest priority item from `tasks/todo.json`
5. **Implement**: Make changes following coding standards
6. **Test**: Run `./scripts/check.sh` for full validation
7. **Update State**: Mark task complete in `tasks/todo.json`, commit
8. **Clean Exit**: Confirm build and tests pass

## Directory Guide

| Directory | Purpose |
|-----------|---------|
| `CLAUDE.md` | Detailed development guide |
| `CONTRIBUTING.md` | Coding standards and contribution guidelines |
| `tasks/` | Task tracking and progress notes |
| `scripts/` | Development automation scripts |
| `include/ccv/` | Public headers (cv/, maths/, kd/) |
| `src/` | Implementation files |
| `unit_test/` | GTest-based unit tests |
| `python/` | Python package (PyPI: libccv) |
| `docs/` | MkDocs documentation |

## Quick Commands

```bash
# Check project status (run this first!)
./scripts/status.sh

# Initialize environment
./scripts/init.sh

# Build the project
./scripts/build.sh

# Run all tests
./scripts/test.sh

# Full check (format, build, test)
./scripts/check.sh

# Format code
./scripts/format.sh

# Clean build artifacts
./scripts/clean.sh
```

## Task Management

- **Active tasks**: `tasks/todo.json`
- **Completed tasks**: `tasks/done.json`
- **Progress notes**: `tasks/progress.md`

## Golden Rules

1. **One task per session** - Don't start multiple features
2. **Test before commit** - All tests must pass
3. **Format before commit** - Run `./scripts/format.sh`
4. **Commit descriptively** - Clear commit messages
5. **Update AGENTS.md** - If you discover new patterns

## Emergency Recovery

```bash
# If build is broken
git status                    # Check current state
git diff                      # Review changes
./scripts/clean.sh            # Clean build artifacts
./scripts/init.sh             # Re-initialize
./scripts/test.sh             # Verify baseline
```

## CI/CD

GitHub Actions workflows:
- `ci.yml` - Build and test on Linux/macOS
- `docs.yml` - Deploy MkDocs to GitHub Pages

## Recovery & Troubleshooting

```bash
# If build is broken
git status                    # Check current state
git diff                      # Review changes
./scripts/clean.sh            # Clean build artifacts
./scripts/init.sh             # Re-initialize
./scripts/test.sh             # Verify baseline

# If tests fail after changes
git log --oneline -5          # Check recent commits
git bisect start              # Find bad commit
```

---

**For detailed information, see [CLAUDE.md](CLAUDE.md)**
