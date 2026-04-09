# Development Progress Notes

## Session Log

### 2026-04-09 - Harness Engineering Setup

**Accomplished:**
- Created AGENTS.md as entry point for AI agents
- Set up task tracking system (todo.json, done.json)
- Created development automation scripts:
  - init.sh - Environment initialization
  - build.sh - CMake build automation
  - test.sh - Test runner
  - format.sh - Code formatting
  - check.sh - Pre-commit validation
  - clean.sh - Build cleanup
- Created GitHub Actions CI/CD workflow
- Added CMakePresets.json for easier configuration

**Decisions:**
- Using JSON for task tracking (more reliable than markdown for programmatic access)
- Scripts placed in scripts/ directory with consistent interface
- Pre-commit check validates: formatting, build, tests

**Next Steps:**
1. Complete SVD decomposition implementation
2. Add more unit test coverage
3. Implement ORB feature detection
4. Document estimation module (EKF, BA)

---

## Architectural Decisions

### Code Organization
- Core library: Zero dependencies (self-contained)
- Wrappers: OpenCV, PCL integration in libs/
- Apps: Standalone applications in apps/
- Python: Separate package with optional dependencies

### Testing Strategy
- C++: GTest with OpenCV for data I/O
- Python: pytest with standard fixtures
- Benchmarks: Compare against Eigen, OpenCV

### Documentation
- Code: Doxygen with LaTeX formulas
- User docs: MkDocs with Material theme
- Developer: AGENTS.md + CLAUDE.md

---

## Known Issues

1. hpc/ directory is currently empty - planned for GPU acceleration
2. Some estimation algorithms need more comprehensive tests
3. Python bindings could be expanded (currently basic)

## Performance Notes

- Matrix operations use custom implementation (not Eigen in core)
- FLOAT macro allows single/double precision toggle
- Consider OpenMP parallelization for large matrices

