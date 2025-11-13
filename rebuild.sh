#!/usr/bin/env bash
set -euo pipefail

# Root of the project = directory containing this script
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${ROOT_DIR}/build"

echo "[rebuild] Root:  ${ROOT_DIR}"
echo "[rebuild] Build: ${BUILD_DIR}"

echo "[rebuild] Removing old build directory..."
rm -rf "${BUILD_DIR}"

echo "[rebuild] Creating fresh build directory..."
mkdir -p "${BUILD_DIR}"
cd "${BUILD_DIR}"

echo "[rebuild] Running CMake configure..."
cmake .. -DBUILD_COMMONROAD_TESTS=ON

# Detect core count if possible, fall back to 8
if command -v sysctl >/dev/null 2>&1; then
    JOBS="$(sysctl -n hw.ncpu)"
elif command -v nproc >/dev/null 2>&1; then
    JOBS="$(nproc)"
else
    JOBS=8
fi

echo "[rebuild] Building with ${JOBS} jobs..."
cmake --build . -- -j"${JOBS}"

echo "[rebuild] Done. Targets available:"
echo "  - commonroad_app"
echo "  - test_derivatives (since BUILD_COMMONROAD_TESTS=ON)"

