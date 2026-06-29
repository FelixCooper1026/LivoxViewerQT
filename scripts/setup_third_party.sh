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

install_tar_dependency() {
    local name="$1"
    local url="$2"
    local archive_name="$3"
    local install_dir_name="$4"
    local required_file="$5"

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

    local source_root
    source_root="$(find "$extract_root" -mindepth 1 -maxdepth 1 -type d | head -n 1)"
    if [[ -z "$source_root" ]]; then
        echo "Archive did not contain an extracted source directory: $archive_path" >&2
        exit 1
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

echo "Third-party dependencies are ready."
