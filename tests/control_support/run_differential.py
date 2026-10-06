"""Compare deterministic host traces with a saved Vector firmware source ZIP.

The ZIP and the working tree must use the current ``algorithm/``, ``src/`` and
``tests/`` layout. Each tree is configured and built through its own CMake test
project, so the comparison exercises exactly the sources and fixtures stored in
that tree.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import shutil
import subprocess
import sys
import zipfile


CONTROL_VARIANTS = (
    ("control-default", "test_runner_control_modes", ()),
    ("control-current", "test_runner_control_modes", ("TORQUE_AND_CURRENT=1",)),
    (
        "control-adjust",
        "test_runner_control_modes",
        ("TORQUE_AND_CURRENT=1", "TORQUE_ADJUST=1"),
    ),
    ("feedforward-cogging", "test_runner_ff_cogging", ()),
)
CALIBRATION_VARIANTS = (
    ("electrical-calibration", "test_runner_electrical_calibration", ()),
)
ARCHIVE_PREFIXES = ("algorithm", "src", "tests")


def parse_args(root: Path) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", required=True, type=Path)
    parser.add_argument(
        "--compiler",
        default="gcc" if shutil.which("gcc") else "clang",
        help="GCC, Clang, command name, or compiler executable path",
    )
    parser.add_argument(
        "--build-dir",
        type=Path,
        default=root / "build" / "control-differential",
    )
    parser.add_argument(
        "--suite", choices=("control", "calibration", "all"), default="all"
    )
    return parser.parse_args()


def resolve_program(value: str, friendly_names: dict[str, str]) -> Path:
    command = friendly_names.get(value.lower(), value)
    resolved = shutil.which(command)
    path = Path(resolved or command).expanduser().resolve()
    if not path.is_file():
        raise ValueError(f"Executable not found: {value}")
    return path


def reset_directory(path: Path, build_root: Path) -> None:
    resolved = path.resolve()
    root = build_root.resolve()
    if resolved == root or root not in resolved.parents:
        raise ValueError(f"Refusing to reset path outside build directory: {resolved}")
    if resolved.exists():
        shutil.rmtree(resolved)
    resolved.mkdir(parents=True)


def extract_baseline(archive_path: Path, destination: Path, build_root: Path) -> None:
    reset_directory(destination, build_root)
    with zipfile.ZipFile(archive_path) as archive:
        for entry in archive.infolist():
            name = entry.filename
            if entry.is_dir() or "\\" in name:
                continue
            relative = PurePosixPath(name)
            if relative.is_absolute() or ".." in relative.parts:
                raise ValueError(f"Unsafe archive path: {name}")
            if not relative.parts or relative.parts[0] not in ARCHIVE_PREFIXES:
                continue
            target = destination.joinpath(*relative.parts)
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(archive.read(entry))

    required = (
        destination / "algorithm" / "foc_current_loop.c",
        destination / "src" / "foc" / "control_dispatcher.c",
        destination / "tests" / "CMakeLists.txt",
    )
    missing = [str(path.relative_to(destination)) for path in required if not path.is_file()]
    if missing:
        raise ValueError("Baseline ZIP does not contain the current layout: " + ", ".join(missing))


def command_environment(compiler: Path) -> dict[str, str]:
    environment = os.environ.copy()
    if os.name == "nt":
        # MinGW/LLVM executables may load compiler runtime DLLs beside the compiler.
        environment["PATH"] = str(compiler.parent) + os.pathsep + environment.get("PATH", "")
    return environment


def run_logged(command: list[str], log: Path, environment: dict[str, str]) -> bytes:
    result = subprocess.run(
        command,
        capture_output=True,
        check=False,
        env=environment,
    )
    with log.open("ab") as stream:
        stream.write(("COMMAND " + json.dumps(command) + "\n").encode("utf-8"))
        stream.write(result.stdout)
        stream.write(result.stderr)
    if result.returncode:
        raise RuntimeError(f"Command failed ({result.returncode}); see {log}")
    return result.stdout


def configure_and_build(
    source_root: Path,
    build_dir: Path,
    target: str,
    definitions: tuple[str, ...],
    compiler: Path,
    cmake: Path,
    ninja: Path,
    log: Path,
    environment: dict[str, str],
) -> Path:
    reset_directory(build_dir, build_dir.parent)
    c_flags = " ".join(f"-D{definition}" for definition in definitions)
    configure = [
        str(cmake),
        "-S",
        str(source_root / "tests"),
        "-B",
        str(build_dir),
        "-G",
        "Ninja",
        f"-DCMAKE_MAKE_PROGRAM={ninja}",
        f"-DCMAKE_C_COMPILER={compiler}",
        "-DCMAKE_BUILD_TYPE=Release",
        f"-DCMAKE_C_FLAGS={c_flags}",
    ]
    run_logged(configure, log, environment)
    run_logged(
        [str(cmake), "--build", str(build_dir), "--target", target, "--parallel"],
        log,
        environment,
    )
    suffix = ".exe" if os.name == "nt" else ""
    executable = build_dir / f"{target}{suffix}"
    if not executable.is_file():
        raise RuntimeError(f"CMake built {target}, but {executable} was not found")
    return executable


def first_difference(left: bytes, right: bytes) -> int | None:
    left_lines = left.splitlines()
    right_lines = right.splitlines()
    for index, (old, new) in enumerate(zip(left_lines, right_lines), 1):
        if old != new:
            return index
    if len(left_lines) != len(right_lines):
        return min(len(left_lines), len(right_lines)) + 1
    return None


def trace_summary(trace: bytes) -> str:
    lines = [line for line in trace.splitlines() if line]
    return lines[-1].decode("utf-8", errors="replace") if lines else "empty trace"


def selected_variants(suite: str):
    if suite == "control":
        return CONTROL_VARIANTS
    if suite == "calibration":
        return CALIBRATION_VARIANTS
    return CONTROL_VARIANTS + CALIBRATION_VARIANTS


def main() -> int:
    root = Path(__file__).resolve().parents[2]
    args = parse_args(root)
    baseline_archive = args.baseline.expanduser().resolve()
    if not baseline_archive.is_file():
        raise ValueError(f"Baseline ZIP not found: {baseline_archive}")

    compiler = resolve_program(args.compiler, {"gcc": "gcc", "clang": "clang"})
    cmake = resolve_program("cmake", {})
    ninja = resolve_program("ninja", {})
    environment = command_environment(compiler)

    build_root = args.build_dir.expanduser().resolve()
    build_root.mkdir(parents=True, exist_ok=True)
    log = build_root / "validation.log"
    log.write_text("Vector deterministic trace comparison\n", encoding="utf-8")

    baseline_hash = hashlib.sha256(baseline_archive.read_bytes()).hexdigest()
    baseline_root = build_root / f"baseline-{baseline_hash[:12]}"
    extract_baseline(baseline_archive, baseline_root, build_root)
    traces_dir = build_root / "traces"
    reset_directory(traces_dir, build_root)

    result = {
        "baseline": str(baseline_archive),
        "baseline_sha256": baseline_hash,
        "compiler": str(compiler),
        "suite": args.suite,
        "comparison": "Exact stdout bytes from each executable's --trace mode",
        "limitations": "Host tests use mocked hardware and do not validate interrupt timing or powered PWM",
        "variants": [],
    }

    for name, target, definitions in selected_variants(args.suite):
        traces: dict[str, bytes] = {}
        for label, source_root in (("baseline", baseline_root), ("current", root)):
            variant_build = build_root / "cmake" / f"{label}-{name}"
            executable = configure_and_build(
                source_root,
                variant_build,
                target,
                definitions,
                compiler,
                cmake,
                ninja,
                log,
                environment,
            )
            trace = run_logged([str(executable), "--trace"], log, environment)
            traces[label] = trace
            (traces_dir / f"{label}-{name}.trace").write_bytes(trace)

        baseline_trace = traces["baseline"]
        current_trace = traces["current"]
        matched = baseline_trace == current_trace
        record = {
            "name": name,
            "target": target,
            "definitions": list(definitions),
            "matched": matched,
            "baseline_sha256": hashlib.sha256(baseline_trace).hexdigest(),
            "current_sha256": hashlib.sha256(current_trace).hexdigest(),
            "trace_lines": len(current_trace.splitlines()),
            "summary": trace_summary(current_trace),
        }
        if not matched:
            record["first_different_line"] = first_difference(
                baseline_trace, current_trace
            )
        result["variants"].append(record)
        print(f"{name}: {'MATCH' if matched else 'DIFFERENT'}; {record['summary']}")

    result["passed"] = all(record["matched"] for record in result["variants"])
    (build_root / "validation.json").write_text(
        json.dumps(result, indent=2) + "\n", encoding="utf-8"
    )
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError, ValueError, zipfile.BadZipFile) as error:
        print(f"error: {error}", file=sys.stderr)
        raise SystemExit(2)
