# Rust modularization reference

This document records the Rust projects reviewed for architecture ideas and the decision for this C firmware.

## Decision

Do not rewrite VectorFOC in Rust now.

Use the Rust projects as architecture references:

- keep a hardware-independent core;
- put board/MCU/peripheral code behind platform adapters;
- keep host or virtual-motor tests close to the core;
- measure flash, RAM and ISR timing before accepting any new abstraction.

For this repository, the near-term implementation remains C. The first Rust experiment, if any, should be a non-ISR `no_std` static library with a narrow C ABI. Candidate pilots:

- protocol frame codec;
- parameter validation;
- generated lookup/table validation;
- offline calibration helper code that does not run in the 20 kHz ISR.

A Rust pilot must pass all gates before it can stay in firmware:

| Gate | Requirement |
|---|---|
| ABI | C callers use an explicit `extern "C"` API with fixed-size types and no Rust allocation visible across the boundary |
| Runtime | no heap, no panic unwind, no blocking path in ISR-adjacent code |
| Size | same four ARM matrix builds compare text/data/bss against the C baseline |
| Timing | 20 kHz loop worst-case time is measured on target before and after |
| Tests | host unit tests plus existing C integration tests pass |
| Rollback | C implementation remains available until the Rust path is proven on hardware |

## Reviewed references

| Project / document | What is useful for VectorFOC | Limitations for this repo |
|---|---|---|
| [OxiFOC](https://github.com/okhsunrog/oxifoc) | Strong reference for `oxifoc-core` as `no_std` hardware-free logic, separate STM32 platform crates, host tools and virtual motor tests | Different stack and runtime choices; no evidence that a direct rewrite would fit current VectorFOC flash/RAM headroom |
| [qff233/FOC](https://github.com/qff233/FOC) | Simple split between `crates/foc` and an STM32 platform crate; useful shape for core/platform separation | Small project surface, no release record; use as design inspiration, not production maturity proof |
| [calebfletcher/foc](https://github.com/calebfletcher/foc) | Algorithm-focused Rust crate with fixed-point goals, generic angle/current/PWM abstractions and no-heap goal | It is a library reference, not a drop-in replacement for this board firmware |
| [Embedded Rust Book: `no_std`](https://docs.rust-embedded.org/book/intro/no-std.html) | Confirms embedded Rust can run without the standard library | `no_std` removes the standard library; it does not automatically make code smaller |
| [Embedded Rust Book: Rust with C](https://docs.rust-embedded.org/book/interoperability/rust-with-c.html) | Describes the supported C/Rust interop direction for incremental adoption | Interop still requires explicit ABI ownership, build integration and panic/allocation policy |
| [Cargo profiles](https://doc.rust-lang.org/cargo/reference/profiles.html) | Provides the knobs for `opt-level`, LTO and panic behavior used in size experiments | Profile tuning must be measured with this firmware’s link script and toolchain |

## What we already borrowed

The PositionSensor refactor follows the same idea without changing language:

```text
ALGO / APP / SAFE
        |
        v
PositionSensor public API
        |
        v
private selected adapter
        |
        v
MT6816 / TMR3109 / Hall / ABZ driver
```

This is the same architectural direction as “core + platform adapter”:

- the 20 kHz control path receives a coherent sample from one boundary;
- sensor capabilities are explicit flags, not scattered preprocessor checks;
- calibration persistence is a portable snapshot;
- raw calibration is typed and only available when supported;
- architecture tests prevent upper layers from learning concrete sensor names.

## Lightweight does not mean “Rust by default”

Rust can be lightweight when the code is `no_std`, allocation-free, optimized for size and linked carefully. It can also grow firmware if generics, formatting, panic paths or runtime support are pulled in accidentally.

VectorFOC is already close to the memory limits on the VectorFOC G431 combinations. Because of that, the safe policy is:

1. Finish C modular boundaries first.
2. Measure the C baseline.
3. Try one pure, non-ISR Rust staticlib.
4. Compare size and timing.
5. Keep the Rust part only if it improves safety or maintainability without hurting the measured budget.

## Next modularization slice

The next C-side seam should target MCU replacement, not another sensor layer:

```text
Core motor/control/parameter logic
        |
        v
MCU kit / platform service
        |
        v
STM32G4 clocks, GPIO, ADC/PWM ISR, CAN, USB, Flash, reset, Bootloader entry
```

Acceptance criteria for that slice:

- ALGO does not include STM32 headers.
- APP ISR entry code is owned by a board/platform adapter.
- CAN/USB transport setup is behind a platform service.
- Flash and reset/Bootloader handoff have narrow interfaces.
- The four current sensor builds and all 66 registered host tests remain green.
- ARM size and 20 kHz timing are measured before and after.
