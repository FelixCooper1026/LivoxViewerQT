#!/usr/bin/env bash
set -euo pipefail

force=0
if [[ "${1:-}" == "--force" ]]; then
    force=1
elif [[ $# -gt 0 ]]; then
    echo "Usage: bash scripts/setup_third_party.sh [--force]" >&2
    exit 2
fi

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$script_dir/.." && pwd)"
third_party_root="$repo_root/third-party"
download_root="$third_party_root/.downloads"

mkdir -p "$third_party_root" "$download_root"

if ! pkg-config --exists opencv4; then
    echo "OpenCV 4 development files are required (for example: libopencv-dev)." >&2
    exit 1
fi

install_tar_dependency() {
    local name="$1"
    local url="$2"
    local archive_name="$3"
    local install_dir_name="$4"
    local required_file="$5"
    local source_subdirectory="${6:-}"
    local use_extract_root="${7:-0}"

    local install_dir="$third_party_root/$install_dir_name"
    local required_path="$install_dir/$required_file"

    if [[ -f "$required_path" && "$force" -eq 0 ]]; then
        echo "$name already installed: $install_dir"
        return
    fi

    if [[ -d "$install_dir" && "$force" -eq 1 ]]; then
        rm -rf "$install_dir"
    fi

    local archive_path="$download_root/$archive_name"
    local extract_root="$download_root/${archive_name%.tar.gz}"

    if [[ ! -f "$archive_path" ]]; then
        echo "Downloading $name..."
        curl -L "$url" -o "$archive_path"
    fi

    rm -rf "$extract_root"
    mkdir -p "$extract_root"

    echo "Extracting $name..."
    tar -xzf "$archive_path" -C "$extract_root"

    local source_root="$extract_root"
    if [[ "$use_extract_root" -eq 0 ]]; then
        source_root="$(find "$extract_root" -mindepth 1 -maxdepth 1 -type d | head -n 1)"
        if [[ -z "$source_root" ]]; then
            echo "Archive did not contain an extracted source directory: $archive_path" >&2
            exit 1
        fi
    fi
    if [[ -n "$source_subdirectory" ]]; then
        source_root="$source_root/$source_subdirectory"
    fi

    mkdir -p "$install_dir"
    find "$source_root" -mindepth 1 -maxdepth 1 -exec mv -t "$install_dir" {} +

    if [[ ! -f "$required_path" ]]; then
        echo "$name installation failed; missing $required_path" >&2
        exit 1
    fi

    echo "$name installed: $install_dir"
}

install_tar_dependency \
    "Eigen 3.4.0" \
    "https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz" \
    "eigen-3.4.0.tar.gz" \
    "eigen-3.4.0" \
    "Eigen/Core"

install_tar_dependency \
    "LZ4 1.10.0" \
    "https://codeload.github.com/lz4/lz4/tar.gz/refs/tags/v1.10.0" \
    "lz4-1.10.0.tar.gz" \
    "lz4-1.10.0" \
    "lz4.c"

install_tar_dependency \
    "MCAP C++ 2.1.3" \
    "https://codeload.github.com/foxglove/mcap/tar.gz/refs/tags/releases/cpp/v2.1.3" \
    "mcap-cpp-2.1.3.tar.gz" \
    "mcap-cpp-2.1.3" \
    "include/mcap/reader.hpp" \
    "cpp/mcap"

install_tar_dependency \
    "Zstd 1.5.7" \
    "https://codeload.github.com/facebook/zstd/tar.gz/refs/tags/v1.5.7" \
    "zstd-1.5.7.tar.gz" \
    "zstd-1.5.7" \
    "build/cmake/CMakeLists.txt"

eigen_root="$third_party_root/eigen-3.4.0"
eigen_config="$eigen_root/share/eigen3/cmake"
mkdir -p "$eigen_config"
printf '%s\n' \
    'set(EIGEN3_FOUND TRUE)' \
    'set(EIGEN3_VERSION "3.4.0")' \
    'set(EIGEN3_VERSION_STRING "3.4.0")' \
    "set(EIGEN3_INCLUDE_DIR \"$eigen_root\")" \
    "set(EIGEN3_INCLUDE_DIRS \"$eigen_root\")" \
    'if(NOT TARGET Eigen3::Eigen)' \
    '    add_library(Eigen3::Eigen INTERFACE IMPORTED)' \
    "    set_target_properties(Eigen3::Eigen PROPERTIES INTERFACE_INCLUDE_DIRECTORIES \"$eigen_root\")" \
    'endif()' \
    > "$eigen_config/Eigen3Config.cmake"

install_tar_dependency \
    "Boost 1.82.0" \
    "https://archives.boost.io/release/1.82.0/source/boost_1_82_0.tar.gz" \
    "boost-1.82.0.tar.gz" \
    "boost-1.82.0" \
    "boost/version.hpp"

install_tar_dependency \
    "GTSAM 4.2.0 source" \
    "https://codeload.github.com/borglab/gtsam/tar.gz/refs/tags/4.2.0" \
    "gtsam-4.2.0.tar.gz" \
    "gtsam-4.2.0" \
    "CMakeLists.txt"

boost_root="$third_party_root/boost-1.82.0"
boost_library_dir="$boost_root/stage/lib"
if ! find "$boost_library_dir" -maxdepth 1 -type f -name '*serialization*' -print -quit 2>/dev/null | grep -q . || [[ "$force" -eq 1 ]]; then
    echo "Building Boost libraries..."
    (
        cd "$boost_root"
        ./bootstrap.sh
        ./b2 \
            --with-serialization \
            --with-system \
            --with-filesystem \
            --with-thread \
            --with-program_options \
            --with-date_time \
            --with-timer \
            --with-chrono \
            --with-regex \
            --layout=tagged \
            variant=debug,release \
            link=static \
            threading=multi \
            stage \
            -j "$(getconf _NPROCESSORS_ONLN)"
    )
fi

gtsam_source="$third_party_root/gtsam-4.2.0"
gtsam_build="$third_party_root/.build/gtsam-4.2.0"
gtsam_install="$third_party_root/gtsam-4.2.0-install"
gtsam_config="$gtsam_install/lib/cmake/GTSAM/GTSAMConfig.cmake"
if [[ ! -f "$gtsam_config" || "$force" -eq 1 ]]; then
    if [[ "$force" -eq 1 ]]; then
        rm -rf "$gtsam_build" "$gtsam_install"
    fi
    cmake \
        -S "$gtsam_source" \
        -B "$gtsam_build" \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="$gtsam_install" \
        -DEigen3_DIR="$third_party_root/eigen-3.4.0/share/eigen3/cmake" \
        -DBOOST_ROOT="$boost_root" \
        -DBOOST_LIBRARYDIR="$boost_library_dir" \
        -DBoost_DIR="$boost_library_dir/cmake/Boost-1.82.0" \
        -DBoost_USE_STATIC_LIBS=ON \
        -DCMAKE_POLICY_DEFAULT_CMP0167=NEW \
        -DCMAKE_POLICY_DEFAULT_CMP0057=NEW \
        -DBoost_NO_SYSTEM_PATHS=ON \
        -DGTSAM_USE_SYSTEM_EIGEN=ON \
        -DGTSAM_BUILD_TESTS=OFF \
        -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
        -DGTSAM_BUILD_UNSTABLE=OFF \
        -DGTSAM_BUILD_PYTHON=OFF \
        -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
        -DBUILD_SHARED_LIBS=OFF \
        -DGTSAM_WITH_TBB=OFF
    cmake --build "$gtsam_build" --target install --parallel
fi

echo "Third-party dependencies are ready."
