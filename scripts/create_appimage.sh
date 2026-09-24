#!/usr/bin/env bash
set -Eeuo pipefail

# ==============================================================================
# LivoxViewerQT Linux AppImage package script
#
# Usage:
#   ./create_appimage.sh
#   ./create_appimage.sh clean
#
# Environment overrides:
#   QT_DIR=<Qt installation prefix> ./create_appimage.sh
#   SOURCE_DIR=/path/to/LivoxViewerQT ./create_appimage.sh
#   APPIMAGETOOL=/path/to/appimagetool ./create_appimage.sh
# ==============================================================================

APP="LivoxViewerQT"
CAPTURE_HELPER="LivoxPacketCaptureHelper"
ICON="livoxviewerqt"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

SOURCE_DIR="${SOURCE_DIR:-$ROOT_DIR}"
QT_DIR="${QT_DIR:-${QT_ROOT_DIR:-}}"

BUILD_DIR="${BUILD_DIR:-$ROOT_DIR/build/cmd-linux-package}"
DIST_DIR="${DIST_DIR:-$ROOT_DIR/dist/linux}"
APPDIR="${APPDIR:-$DIST_DIR/AppDir}"

INSTALL_PREFIX="/usr"
BUILD_TYPE="Release"
CLEAN_BUILD=0

usage()
{
    cat <<EOF
Usage:
  ./create_appimage.sh [clean]

Environment:
  QT_DIR=<Qt installation prefix> ./create_appimage.sh
  SOURCE_DIR=/path/to/LivoxViewerQT ./create_appimage.sh
  APPIMAGETOOL=/path/to/appimagetool ./create_appimage.sh
EOF
}

log()
{
    echo
    echo "==== $* ===="
}

warn()
{
    echo "Warning: $*" >&2
}

die()
{
    echo "Error: $*" >&2
    exit 1
}

require_cmd()
{
    command -v "$1" >/dev/null 2>&1 || die "Command not found: $1"
}

for arg in "$@"; do
    case "$arg" in
        clean|--clean)
            CLEAN_BUILD=1
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown argument: $arg"
            usage
            exit 1
            ;;
    esac
done

log "Check environment"

[ -f "$SOURCE_DIR/CMakeLists.txt" ] || die "CMakeLists.txt not found in SOURCE_DIR: $SOURCE_DIR"
[ -n "$QT_DIR" ] || die "Set QT_DIR or QT_ROOT_DIR to the Qt installation prefix"

require_cmd cmake
require_cmd python3
require_cmd nproc
require_cmd ldd
require_cmd find
require_cmd grep
require_cmd linuxdeployqt

[ -d "$QT_DIR" ] || die "QT_DIR does not exist: $QT_DIR"
[ -x "$QT_DIR/bin/qmake" ] || die "qmake not found or not executable: $QT_DIR/bin/qmake"
[ -f "$QT_DIR/lib/libQt6Core.so.6" ] || die "libQt6Core.so.6 not found: $QT_DIR/lib/libQt6Core.so.6"

if ! command -v patchelf >/dev/null 2>&1; then
    warn "patchelf not found. Recommended: sudo apt install -y patchelf"
fi

if ! command -v desktop-file-validate >/dev/null 2>&1; then
    warn "desktop-file-validate not found. Recommended: sudo apt install -y desktop-file-utils"
fi

log "Setup Qt environment"

export PATH="$QT_DIR/bin:$PATH"
export LD_LIBRARY_PATH="$QT_DIR/lib:${LD_LIBRARY_PATH:-}"
export QT_PLUGIN_PATH="$QT_DIR/plugins"
export QML2_IMPORT_PATH="$QT_DIR/qml"

echo "SOURCE_DIR: $SOURCE_DIR"
echo "BUILD_DIR:  $BUILD_DIR"
echo "DIST_DIR:   $DIST_DIR"
echo "APPDIR:     $APPDIR"
echo "QT_DIR:     $QT_DIR"
echo "qmake:      $(command -v qmake)"
qmake -v || true
echo "linuxdeployqt: $(command -v linuxdeployqt)"
linuxdeployqt --version || true

log "Read project version"

APP_VERSION="$(python3 - "$SOURCE_DIR/CMakeLists.txt" <<'PY'
import re
import sys
from pathlib import Path

cmake_file = Path(sys.argv[1])
text = cmake_file.read_text(encoding="utf-8", errors="ignore")

m = re.search(
    r'project\s*\(\s*LivoxViewerQT\b.*?\bVERSION\s+([0-9]+(?:\.[0-9]+){0,3})',
    text,
    re.S
)

if not m:
    raise SystemExit("Failed to read project VERSION from CMakeLists.txt")

print(m.group(1))
PY
)"

ARCH="$(uname -m)"
APPIMAGE_NAME="${APP}-${APP_VERSION}-${ARCH}.AppImage"
APPIMAGE_PATH="$DIST_DIR/$APPIMAGE_NAME"
APPIMAGE_CACHE_DIR="${XDG_CACHE_HOME:-$HOME/.cache}/livoxviewerqt/appimage"
APPIMAGETOOL="${APPIMAGETOOL:-$(command -v appimagetool || true)}"
APPIMAGETOOL="${APPIMAGETOOL:-$APPIMAGE_CACHE_DIR/appimagetool-$ARCH.AppImage}"
APPIMAGETOOL_URL="${APPIMAGETOOL_URL:-https://github.com/AppImage/AppImageKit/releases/download/continuous/appimagetool-$ARCH.AppImage}"
APPIMAGE_RUNTIME_FILE="${APPIMAGE_RUNTIME_FILE:-$APPIMAGE_CACHE_DIR/runtime-$ARCH}"
APPIMAGE_RUNTIME_URL="${APPIMAGE_RUNTIME_URL:-https://github.com/AppImage/type2-runtime/releases/download/continuous/runtime-$ARCH}"

if [ ! -f "$APPIMAGETOOL" ]; then
    require_cmd curl
    mkdir -p "$(dirname "$APPIMAGETOOL")"
    log "Download appimagetool"
    echo "URL: $APPIMAGETOOL_URL"
    curl --fail --location \
        --retry 3 \
        --output "$APPIMAGETOOL.download" \
        "$APPIMAGETOOL_URL" \
        || die "Failed to download appimagetool: $APPIMAGETOOL_URL"
    mv "$APPIMAGETOOL.download" "$APPIMAGETOOL"
fi
chmod +x "$APPIMAGETOOL"

if [ ! -f "$APPIMAGE_RUNTIME_FILE" ]; then
    require_cmd curl
    mkdir -p "$(dirname "$APPIMAGE_RUNTIME_FILE")"
    log "Download AppImage runtime"
    echo "URL: $APPIMAGE_RUNTIME_URL"
    curl --fail --location \
        --retry 3 \
        --output "$APPIMAGE_RUNTIME_FILE.download" \
        "$APPIMAGE_RUNTIME_URL" \
        || die "Failed to download AppImage runtime: $APPIMAGE_RUNTIME_URL"
    mv "$APPIMAGE_RUNTIME_FILE.download" "$APPIMAGE_RUNTIME_FILE"
fi

echo "App version: $APP_VERSION"
echo "Target AppImage: $APPIMAGE_PATH"
echo "appimagetool: $APPIMAGETOOL"
echo "AppImage runtime: $APPIMAGE_RUNTIME_FILE"

log "Clean package directories"

mkdir -p "$DIST_DIR"
rm -rf "$APPDIR"
rm -f "$APPIMAGE_PATH"
rm -f "$DIST_DIR"/*.AppImage

if [ "$CLEAN_BUILD" = "1" ]; then
    echo "Clean build enabled: remove $BUILD_DIR"
    rm -rf "$BUILD_DIR"
fi

log "Configure"

cmake -S "$SOURCE_DIR" -B "$BUILD_DIR" \
    -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
    -DCMAKE_INSTALL_PREFIX="$INSTALL_PREFIX" \
    -DCMAKE_PREFIX_PATH="$QT_DIR"

log "Build"

cmake --build "$BUILD_DIR" --target "$APP" "$CAPTURE_HELPER" --parallel "${BUILD_JOBS:-$(nproc)}"

log "Install to AppDir"

DESTDIR="$APPDIR" cmake --install "$BUILD_DIR"

log "Validate executables"

APP_EXE="$APPDIR/usr/bin/$APP"
[ -x "$APP_EXE" ] || die "Executable not found or not executable: $APP_EXE"

CAPTURE_HELPER_EXE="$APPDIR/usr/bin/$CAPTURE_HELPER"
[ -x "$CAPTURE_HELPER_EXE" ] || die "Capture helper not found or not executable: $CAPTURE_HELPER_EXE"

log "Validate or create desktop file"

DESKTOP_DIR="$APPDIR/usr/share/applications"
DESKTOP_FILE="$DESKTOP_DIR/${APP}.desktop"

mkdir -p "$DESKTOP_DIR"

if [ ! -f "$DESKTOP_FILE" ]; then
    warn "Desktop file not installed by CMake. Creating fallback: $DESKTOP_FILE"

    cat > "$DESKTOP_FILE" <<EOF
[Desktop Entry]
Type=Application
Name=${APP}
Comment=Livox LiDAR Qt Viewer Application
Exec=${APP}
Icon=${ICON}
Terminal=false
Categories=Utility;Graphics;
StartupWMClass=${APP}
EOF
fi

grep -q "^Type=Application" "$DESKTOP_FILE" || die "Invalid desktop file: missing Type=Application"
grep -q "^Name=${APP}" "$DESKTOP_FILE" || die "Invalid desktop file: missing Name=${APP}"
grep -q "^Icon=${ICON}" "$DESKTOP_FILE" || die "Desktop Icon must be '${ICON}', without .png suffix"

if command -v desktop-file-validate >/dev/null 2>&1; then
    desktop-file-validate "$DESKTOP_FILE" || true
fi

echo "Desktop file:"
cat "$DESKTOP_FILE"

log "Validate or install icon"

ICON_SRC="$SOURCE_DIR/resources/app_icon.png"
ICON_FILE="$APPDIR/usr/share/icons/hicolor/256x256/apps/${ICON}.png"
PIXMAP_ICON_FILE="$APPDIR/usr/share/pixmaps/${ICON}.png"

mkdir -p "$(dirname "$ICON_FILE")"
mkdir -p "$(dirname "$PIXMAP_ICON_FILE")"

if [ ! -f "$ICON_FILE" ]; then
    if [ -f "$ICON_SRC" ]; then
        warn "Icon was not installed by CMake. Copying fallback icon from: $ICON_SRC"
        cp "$ICON_SRC" "$ICON_FILE"
    else
        die "Icon not found: $ICON_FILE, and fallback does not exist: $ICON_SRC"
    fi
fi

if [ ! -f "$PIXMAP_ICON_FILE" ]; then
    cp "$ICON_FILE" "$PIXMAP_ICON_FILE"
fi

file "$ICON_FILE" || true

log "Create AppImage root metadata"

cp "$DESKTOP_FILE" "$APPDIR/${APP}.desktop"

# Use a real PNG file for .DirIcon, not a symlink. This improves compatibility.
rm -f "$APPDIR/.DirIcon"
cp "$ICON_FILE" "$APPDIR/.DirIcon"

# Also keep a root-level icon for tools that look for it.
cp "$ICON_FILE" "$APPDIR/${ICON}.png"

echo "Root metadata:"
find "$APPDIR" -maxdepth 3 \( -iname "*.desktop" -o -iname "*.png" -o -name ".DirIcon" \) -print

log "Precheck dynamic libraries"

echo "ldd $APP_EXE | grep -E 'Qt6|livox|not found'"
ldd "$APP_EXE" | grep -E "Qt6|livox|not found" || true

if ldd "$APP_EXE" | grep -q "not found"; then
    die "Executable has missing dynamic libraries before linuxdeployqt"
fi

echo
echo "ldd $CAPTURE_HELPER_EXE | grep -E 'pcap|not found'"
ldd "$CAPTURE_HELPER_EXE" | grep -E "pcap|not found" || true

if ldd "$CAPTURE_HELPER_EXE" | grep -q "not found"; then
    die "Capture helper has missing dynamic libraries before linuxdeployqt"
fi

# Check Qt library dependency visibility. This catches the common libQt6Core.so.6 => not found problem.
if [ -f "$QT_DIR/lib/libQt6SerialPort.so.6" ]; then
    echo
    echo "Check Qt6SerialPort dependency:"
    ldd "$QT_DIR/lib/libQt6SerialPort.so.6" | grep -E "Qt6Core|not found" || true

    if ldd "$QT_DIR/lib/libQt6SerialPort.so.6" | grep -q "libQt6Core.so.6 => not found"; then
        die "Qt library dependency check failed. LD_LIBRARY_PATH may not include: $QT_DIR/lib"
    fi
fi

log "Deploy AppDir dependencies"

cd "$DIST_DIR"

export VERSION="$APP_VERSION"

linuxdeployqt "$DESKTOP_FILE" \
    -unsupported-allow-new-glibc \
    -executable="$CAPTURE_HELPER_EXE" \
    -bundle-non-qt-libs \
    -verbose=3

TLS_PLUGIN="$QT_DIR/plugins/tls/libqopensslbackend.so"
[ -f "$TLS_PLUGIN" ] || die "Qt OpenSSL TLS plugin not found"
mkdir -p "$APPDIR/usr/plugins/tls" "$APPDIR/usr/lib"
cp -L "$TLS_PLUGIN" "$APPDIR/usr/plugins/tls/"
for lib in libssl libcrypto; do
    dep="$(ldd "$TLS_PLUGIN" | awk -v name="$lib" '$1 ~ "^" name "\\.so\\." { print $3; exit }')"
    [ -f "$dep" ] || die "Qt TLS plugin must be linked to $lib"
    cp -L "$dep" "$APPDIR/usr/lib/"
done

log "Create AppImage launcher"

[ -e "$APPDIR/AppRun" ] || die "linuxdeployqt did not create AppRun"
mv "$APPDIR/AppRun" "$APPDIR/AppRun.livoxviewerqt"

cat > "$APPDIR/AppRun" <<'EOF'
#!/bin/sh
set -e

APPDIR="${APPDIR:-$(dirname "$(readlink -f "$0")")}"

if [ "${1:-}" = "--livox-packet-capture-helper" ]; then
    shift
    exec "$APPDIR/usr/bin/LivoxPacketCaptureHelper" "$@"
fi

exec "$APPDIR/AppRun.livoxviewerqt" "$@"
EOF

chmod +x "$APPDIR/AppRun"

log "Package AppImage with local runtime"

APPIMAGE_EXTRACT_AND_RUN=1 ARCH="$ARCH" "$APPIMAGETOOL" \
    --runtime-file "$APPIMAGE_RUNTIME_FILE" \
    "$APPDIR" \
    "$APPIMAGE_PATH"

[ -f "$APPIMAGE_PATH" ] || die "AppImage was not generated: $APPIMAGE_PATH"

chmod +x "$APPIMAGE_PATH"

log "Final check"

ls -lh "$APPIMAGE_PATH"

echo
echo "Extract check:"
echo "  \"$APPIMAGE_PATH\" --appimage-extract"
echo
echo "Icon check after extract:"
echo "  find squashfs-root \\( -iname '*.desktop' -o -iname '*.png' -o -name '.DirIcon' \\) -print"
echo
echo "Capture helper check after extract:"
echo "  test -x squashfs-root/usr/bin/$CAPTURE_HELPER && ldd squashfs-root/usr/bin/$CAPTURE_HELPER"
echo
echo "Run:"
echo "  \"$APPIMAGE_PATH\""

echo
echo "Done: $APPIMAGE_PATH"
