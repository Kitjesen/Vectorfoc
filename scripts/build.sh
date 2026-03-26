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
    
    mkdir -p "$PROJECT_DIR/build"
    cd "$PROJECT_DIR/build"
    
    cmake -DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN_FILE" \
          -DCMAKE_BUILD_TYPE=Release \
          "$PROJECT_DIR"
    
    make -j$(nproc)
    
    echo ""
    echo "Application built: build/VectorFoc.bin"
}

build_bootloader() {
    echo "=========================================="
    echo "Building Bootloader..."
    echo "=========================================="
    
    mkdir -p "$PROJECT_DIR/build_boot"
    cd "$PROJECT_DIR/build_boot"
    
    # Copy bootloader CMakeLists
    cp "$PROJECT_DIR/CMakeLists_Bootloader.txt" "$PROJECT_DIR/CMakeLists_Boot_Temp.txt"
    
    cmake -DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN_FILE" \
          -DCMAKE_BUILD_TYPE=Release \
          -C "$PROJECT_DIR/CMakeLists_Bootloader.txt" \
          "$PROJECT_DIR"
    
    make -j$(nproc)
    
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
