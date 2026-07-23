#!/usr/bin/env bash
set -euo pipefail

# Usage: compile.sh [config] [build_dir] [target] [jobs]
# Environment overrides: LIVOX_BUILD_CONFIG, LIVOX_BUILD_DIR,
# LIVOX_BUILD_TARGET, LIVOX_BUILD_JOBS, LIVOX_CMAKE_COMMAND, CMAKE_PREFIX_PATH.

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/.." && pwd)"

build_config="${1:-${LIVOX_BUILD_CONFIG:-Release}}"
build_dir="${2:-${LIVOX_BUILD_DIR:-${project_root}/build-linux}}"
build_target="${3:-${LIVOX_BUILD_TARGET:-LivoxViewerQT}}"
build_jobs="${4:-${LIVOX_BUILD_JOBS:-2}}"
cmake_command="${LIVOX_CMAKE_COMMAND:-cmake}"

echo "Configuring ${build_config} in ${build_dir}..."
"${cmake_command}" \
    -S "${project_root}" \
    -B "${build_dir}" \
    -DCMAKE_BUILD_TYPE="${build_config}"

echo "Building ${build_target}..."
"${cmake_command}" \
    --build "${build_dir}" \
    --config "${build_config}" \
    --target "${build_target}" \
    --parallel "${build_jobs}"

echo "Build completed successfully."
