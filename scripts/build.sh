#!/usr/bin/env bash
# VectorFOC offline build helper: ./build.sh [app|boot|all|clean]
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
TOOLCHAIN_FILE="$PROJECT_DIR/cmake/gcc-arm-none-eabi.cmake"

build_app() {
    cmake -S "$PROJECT_DIR" -B "$PROJECT_DIR/build/arm" -G Ninja \
        --toolchain "$TOOLCHAIN_FILE" -DCMAKE_BUILD_TYPE=Release
    cmake --build "$PROJECT_DIR/build/arm" --parallel 4
    echo "Application built: build/arm/VectorFoc.bin"
}

build_bootloader() {
    cmake -S "$PROJECT_DIR/cmake/bootloader" -B "$PROJECT_DIR/build/boot" -G Ninja \
        --toolchain "$TOOLCHAIN_FILE" -DCMAKE_BUILD_TYPE=Release
    cmake --build "$PROJECT_DIR/build/boot" --parallel 4
    echo "Bootloader built: build/boot/VectorFoc_Bootloader.bin"
}

clean_all() {
    for target in arm host boot keil; do
        if [[ -f "$PROJECT_DIR/build/$target/CMakeCache.txt" ]]; then
            cmake --build "$PROJECT_DIR/build/$target" --target clean
        fi
    done
}

case "${1:-app}" in
    app) build_app ;;
    boot) build_bootloader ;;
    all) build_bootloader; build_app ;;
    clean) clean_all ;;
    *) echo "Usage: $0 [app|boot|all|clean]" >&2; exit 1 ;;
esac
