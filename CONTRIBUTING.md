# Contributing to VectorFOC

Thank you for your interest in contributing! This document describes how to submit bug reports, propose features, and send pull requests.

## Code of Conduct

This project follows our [Code of Conduct](CODE_OF_CONDUCT.md). By participating, you agree to uphold it.

## Reporting Bugs

1. Search [existing issues](https://github.com/kitjesen/vectorfoc/issues) first.
2. Open a new issue with:
   - Hardware board and firmware version (`git describe --tags`)
   - Motor type and parameters
   - Steps to reproduce
   - Expected vs. actual behavior
   - Relevant log output (VoFA+ scope trace, CAN frame dump, etc.)

## Requesting Features

Open a GitHub Issue tagged `enhancement`. Describe the use case, not just the solution.

## Pull Requests

### Before You Start

- For significant changes, open an issue first to discuss the approach.
- One pull request per feature or fix.
- All new algorithm code must have corresponding tests under `test/`.

### Development Workflow

```bash
# 1. Fork and clone the repository
git clone https://github.com/<your-fork>/vectorfoc.git
cd vectorfoc

# 2. Create a feature branch
git checkout -b feat/my-feature

# 3. Build and run tests locally
cmake -S test -B build_test && cmake --build build_test -j$(nproc)
ctest --test-dir build_test -V

# 4. Commit with a clear message (see Commit Style below)
git commit -m "feat(foc): add flux-weakening current limit ramp"

# 5. Push and open a PR against main
git push origin feat/my-feature
```

### Commit Style

Follow [Conventional Commits](https://www.conventionalcommits.org/):

```
<type>(<scope>): <short summary>

[optional body]
[optional footer]
```

Types: `feat`, `fix`, `refactor`, `test`, `docs`, `ci`, `chore`

Scopes: `foc`, `motor`, `hal`, `comm`, `config`, `test`, `ci`

### Code Style

- C99 for all embedded code.
- 2-space indentation, 100-column line limit.
- Format with `clang-format` using the project `.clangd` config before committing.
- No dynamic memory allocation (`malloc`/`free`) in embedded paths — use static pools.
- No floating-point divisions in the 20 kHz ISR — precompute reciprocals.
- Algorithm code in `Src/ALGO/` must be platform-independent (no HAL headers, no `#ifdef BOARD_*`).

### Testing Requirements

- Every new algorithm function must have at least one unit test in `test/`.
- All 43 existing tests must continue to pass: `ctest --test-dir build_test -V`.
- CI runs these tests automatically on every push.

## License

By contributing, you agree your contributions will be licensed under the [Apache License 2.0](LICENSE).
