#!/usr/bin/env bash
set -Eeuo pipefail

QT_VERSION=6.8.3
QT_DIR="${QT_DIR:?Set QT_DIR to the Qt installation prefix}"
QT_BUILD_JOBS="${QT_BUILD_JOBS:-2}"
SOURCE_ROOT="${RUNNER_TEMP:-/tmp}/qt-$QT_VERSION-sources"
BUILD_ROOT="${RUNNER_TEMP:-/tmp}/qt-$QT_VERSION-build"
BASE_URL="https://download.qt.io/official_releases/qt/6.8/$QT_VERSION/submodules"

mkdir -p "$SOURCE_ROOT" "$BUILD_ROOT" "$QT_DIR"
export CMAKE_PREFIX_PATH="$QT_DIR"
export PATH="$QT_DIR/bin:$PATH"
export LD_LIBRARY_PATH="$QT_DIR/lib:${LD_LIBRARY_PATH:-}"

for module in qtbase qtsvg qtserialport qtcharts; do
    archive="$module-everywhere-src-$QT_VERSION.tar.xz"
    source_dir="$SOURCE_ROOT/${archive%.tar.xz}"
    build_dir="$BUILD_ROOT/$module"

    curl -fL --retry 3 "$BASE_URL/$archive" -o "$SOURCE_ROOT/$archive"
    tar -xJf "$SOURCE_ROOT/$archive" -C "$SOURCE_ROOT"

    cmake -S "$source_dir" -B "$build_dir" -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="$QT_DIR" \
        -DINPUT_openssl=linked \
        -DQT_BUILD_TESTS=OFF \
        -DQT_BUILD_EXAMPLES=OFF \
        -DQT_GENERATE_SBOM=OFF
    cmake --build "$build_dir" --parallel "$QT_BUILD_JOBS"
    cmake --install "$build_dir"
    rm -rf "$source_dir" "$build_dir" "$SOURCE_ROOT/$archive"
done
