#!/usr/bin/env python3
"""Enforce the public PositionSensor boundary above hardware adapters."""

from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SCANNED_ROOTS = (
    REPO_ROOT / "Src" / "ALGO",
    REPO_ROOT / "Src" / "APP",
    REPO_ROOT / "Src" / "SAFE",
)
FORBIDDEN = (
    "HW_POSITION_SENSOR_MODE",
    "mt6816_encoder.h",
    "tmr3109_encoder.h",
    "hall_encoder.h",
    "abz_encoder.h",
    "MT6816_Handle_t",
    "TMR3109_Handle_t",
    "Hall_Handle_t",
    "Abz_Handle_t",
    "encoder_data",
    "tmr3109_encoder_data",
    "hall_data",
    "abz_data",
)


def main() -> int:
    violations: list[str] = []
    for root in SCANNED_ROOTS:
        for path in sorted(root.rglob("*")):
            if path.suffix not in {".c", ".h"}:
                continue
            for line_number, line in enumerate(
                path.read_text(encoding="utf-8").splitlines(), start=1
            ):
                for token in FORBIDDEN:
                    if token in line:
                        relative = path.relative_to(REPO_ROOT).as_posix()
                        violations.append(
                            f"{relative}:{line_number}: concrete sensor leak: {token}"
                        )

    if violations:
        print("PositionSensor architecture boundary violations:")
        print("\n".join(violations))
        return 1

    print("PositionSensor architecture boundary PASSED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
