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
- All new algorithm code must have corresponding tests under `tests/`.

### Development Workflow

```bash
# 1. Fork and clone the repository
git clone https://github.com/<your-fork>/vectorfoc.git
cd vectorfoc

# 2. Create a feature branch
git checkout -b feat/my-feature

# 3. Build and run tests locally
cmake -S tests -B build/host -G Ninja
cmake --build build/host --parallel 4
ctest --test-dir build/host --output-on-failure

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

Scopes: `foc`, `motor_runtime`, `hal`, `comm`, `config`, `test`, `ci`

### Code Style

- C99 for all embedded code.
- 2-space indentation, 100-column line limit.
- Follow surrounding formatting; `.clangd` configures the language server, not `clang-format`.
- No dynamic memory allocation (`malloc`/`free`) in embedded paths — use static pools.
- Keep the PWM/ADC ISR bounded and non-blocking; measure timing on hardware before increasing its work.
- Pure algorithms live in `algorithm/` without HAL headers or board macros. Runtime scheduling lives in `src/foc/`; hardware access lives in `src/hal/`.

### Testing Requirements

- Every new algorithm function must have at least one unit test in `tests/`.
- Run the affected tests and the default host set described in [the build guide](../docs/BUILD_GUIDE.md). The default build contains 18 test executables, one CubeMX consistency check and 16 compile-time timing configuration checks. Release builds keep test assertions active. Passing the tests does not validate hardware behavior.
- CI runs these tests automatically on every push.

## License

By contributing, you agree your contributions will be licensed under the [Apache License 2.0](../LICENSE).
