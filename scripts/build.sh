#!/bin/bash
# VectorFOC Build Script
# Usage: ./build.sh [app|boot|all|clean]

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
TOOLCHAIN_FILE="$PROJECT_DIR/cmake/gcc-arm-none-eabi.cmake"

build_app() {
    echo "=========================================="
    echo "Building Application..."
    echo "=========================================="
    
    cmake -S "$PROJECT_DIR" -B "$PROJECT_DIR/build" \
          -DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN_FILE" \
          -DCMAKE_BUILD_TYPE=Release \
          -DBOOTLOADER_BUILD=OFF

    cmake --build "$PROJECT_DIR/build" --parallel "$(nproc)"
    
    echo ""
    echo "Application built: build/VectorFoc.bin"
}

build_bootloader() {
    echo "=========================================="
    echo "Building Bootloader..."
    echo "=========================================="
    
    cmake -S "$PROJECT_DIR" -B "$PROJECT_DIR/build_boot" \
          -DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN_FILE" \
          -DCMAKE_BUILD_TYPE=Release \
          -DBOOTLOADER_BUILD=ON

    cmake --build "$PROJECT_DIR/build_boot" --parallel "$(nproc)"
    
    echo ""
    echo "Bootloader built: build_boot/VectorFoc_Bootloader.bin"
}

clean_all() {
    echo "Cleaning build directories..."
    rm -rf "$PROJECT_DIR/build"
    rm -rf "$PROJECT_DIR/build_boot"
    echo "Done."
}

case "${1:-all}" in
    app)
        build_app
        ;;
    boot)
        build_bootloader
        ;;
    all)
        build_bootloader
        build_app
        ;;
    clean)
        clean_all
        ;;
    *)
        echo "Usage: $0 [app|boot|all|clean]"
        echo "  app   - Build application only"
        echo "  boot  - Build bootloader only"
        echo "  all   - Build both (default)"
        echo "  clean - Clean build directories"
        exit 1
        ;;
esac

echo ""
echo "Build complete!"
