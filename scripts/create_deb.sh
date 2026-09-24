#!/usr/bin/env bash
set -Eeuo pipefail

APP="LivoxViewerQT"
CAPTURE_HELPER="LivoxPacketCaptureHelper"
PKG_NAME="livoxviewerqt"
ICON="livoxviewerqt"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

QT_DIR="${QT_DIR:-${QT_ROOT_DIR:-}}"

BUILD_DIR="$ROOT_DIR/build/cmd-linux-deb"
DIST_DIR="$ROOT_DIR/dist/linux"
DEBROOT="$DIST_DIR/debroot"
OPT_DIR="$DEBROOT/opt/$APP"

VERSION="$(python3 - "$ROOT_DIR/CMakeLists.txt" <<'PY'
import re
import sys
from pathlib import Path

text = Path(sys.argv[1]).read_text(encoding="utf-8", errors="ignore")
m = re.search(
    r'project\s*\(\s*LivoxViewerQT\b.*?\bVERSION\s+([0-9]+(?:\.[0-9]+){0,3})',
    text,
    re.S
)

if not m:
    raise SystemExit("Failed to read VERSION from CMakeLists.txt")

print(m.group(1))
PY
)"

ARCH="amd64"
DEB_NAME="${PKG_NAME}_${VERSION}_${ARCH}.deb"

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

copy_lib()
{
    local src="$1"
    local dst_dir="$2"

    if [ -f "$src" ]; then
        local base
        base="$(basename "$src")"

        if [ ! -f "$dst_dir/$base" ]; then
            cp -L "$src" "$dst_dir/"
            echo "Copied: $base"
        fi
    fi
}

copy_ldd_deps()
{
    local target="$1"
    local dst_dir="$2"

    if [ ! -f "$target" ]; then
        return
    fi

    ldd "$target" 2>/dev/null | awk '
        /=>/ && $3 ~ /^\// { print $3 }
        /^[[:space:]]*\// { print $1 }
    ' | while read -r dep; do
        [ -f "$dep" ] || continue

        local base
        base="$(basename "$dep")"

        case "$base" in
            ld-linux*.so.*|linux-vdso.so.*)
                ;;
            libc.so.*|libm.so.*|libpthread.so.*|libdl.so.*|librt.so.*|libresolv.so.*)
                ;;
            libstdc++.so.*|libgcc_s.so.*)
                # Bundle the C++ runtime libraries used by the build.
                copy_lib "$dep" "$dst_dir"
                ;;
            *)
                copy_lib "$dep" "$dst_dir"
                ;;
        esac
    done
}

patch_rpath()
{
    local file="$1"
    local rpath="$2"

    if command -v patchelf >/dev/null 2>&1 && [ -f "$file" ]; then
        patchelf --set-rpath "$rpath" "$file" || true
    fi
}

if [ "${1:-}" = "clean" ] || [ "${1:-}" = "--clean" ]; then
    rm -rf "$BUILD_DIR"
fi

log "Environment"

[ -f "$ROOT_DIR/CMakeLists.txt" ] || die "CMakeLists.txt not found: $ROOT_DIR"
[ -n "$QT_DIR" ] || die "Set QT_DIR or QT_ROOT_DIR to the Qt installation prefix"
[ -d "$QT_DIR" ] || die "QT_DIR not found: $QT_DIR"
[ -d "$QT_DIR/lib" ] || die "Qt lib directory not found: $QT_DIR/lib"
[ -d "$QT_DIR/plugins" ] || die "Qt plugins directory not found: $QT_DIR/plugins"

command -v cmake >/dev/null || die "cmake not found"
command -v dpkg-deb >/dev/null || die "dpkg-deb not found"
command -v python3 >/dev/null || die "python3 not found"

if ! command -v patchelf >/dev/null 2>&1; then
    warn "patchelf not found. Recommended: sudo apt install -y patchelf"
fi

export PATH="$QT_DIR/bin:$PATH"
export LD_LIBRARY_PATH="$QT_DIR/lib:${LD_LIBRARY_PATH:-}"

echo "ROOT_DIR:  $ROOT_DIR"
echo "QT_DIR:    $QT_DIR"
echo "VERSION:   $VERSION"
echo "BUILD_DIR: $BUILD_DIR"
echo "DEBROOT:   $DEBROOT"
echo "OUTPUT:    $DIST_DIR/$DEB_NAME"

log "Configure"

cmake -S "$ROOT_DIR" -B "$BUILD_DIR" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH="$QT_DIR"

log "Build"

cmake --build "$BUILD_DIR" --target "$APP" "$CAPTURE_HELPER" -j"$(nproc)"

APP_EXE="$BUILD_DIR/$APP"

if [ ! -x "$APP_EXE" ]; then
    APP_EXE="$(find "$BUILD_DIR" -maxdepth 4 -type f -name "$APP" -executable | head -n 1 || true)"
fi

[ -x "$APP_EXE" ] || die "Executable not found: $APP"

CAPTURE_HELPER_EXE="$BUILD_DIR/$CAPTURE_HELPER"

if [ ! -x "$CAPTURE_HELPER_EXE" ]; then
    CAPTURE_HELPER_EXE="$(find "$BUILD_DIR" -maxdepth 4 -type f -name "$CAPTURE_HELPER" -executable | head -n 1 || true)"
fi

[ -x "$CAPTURE_HELPER_EXE" ] || die "Executable not found: $CAPTURE_HELPER"

echo "Executable: $APP_EXE"
echo "Capture helper: $CAPTURE_HELPER_EXE"

log "Clean debroot"

rm -rf "$DEBROOT"
mkdir -p "$OPT_DIR"
mkdir -p "$OPT_DIR/lib"
mkdir -p "$OPT_DIR/plugins"
mkdir -p "$DEBROOT/DEBIAN"
mkdir -p "$DEBROOT/usr/bin"
mkdir -p "$DEBROOT/usr/share/applications"
mkdir -p "$DEBROOT/usr/share/icons/hicolor/256x256/apps"

log "Copy application files"

cp "$APP_EXE" "$OPT_DIR/$APP"
cp "$CAPTURE_HELPER_EXE" "$OPT_DIR/$CAPTURE_HELPER"
chmod 755 "$OPT_DIR/$CAPTURE_HELPER"

[ -f "$ROOT_DIR/config.json" ] && cp "$ROOT_DIR/config.json" "$OPT_DIR/"
[ -f "$ROOT_DIR/mid360_config.json" ] && cp "$ROOT_DIR/mid360_config.json" "$OPT_DIR/"

if [ -d "$ROOT_DIR/livox_sdk_qt" ]; then
    cp -a "$ROOT_DIR/livox_sdk_qt" "$OPT_DIR/"
fi

[ -f "$ROOT_DIR/resources/app_icon.png" ] || die "Missing icon: resources/app_icon.png"

cp "$ROOT_DIR/resources/app_icon.png" \
   "$DEBROOT/usr/share/icons/hicolor/256x256/apps/${ICON}.png"

log "Copy Qt libraries"

# Copy libraries required by the application and capture helper.
copy_ldd_deps "$APP_EXE" "$OPT_DIR/lib"
copy_ldd_deps "$CAPTURE_HELPER_EXE" "$OPT_DIR/lib"

# Include the Qt libraries used by the application.
for lib in \
    libQt6Core.so.6 \
    libQt6Gui.so.6 \
    libQt6Widgets.so.6 \
    libQt6OpenGL.so.6 \
    libQt6OpenGLWidgets.so.6 \
    libQt6SerialPort.so.6 \
    libQt6Charts.so.6 \
    libQt6Network.so.6 \
    libQt6Svg.so.6 \
    libQt6DBus.so.6 \
    libQt6XcbQpa.so.6
do
    if [ -f "$QT_DIR/lib/$lib" ]; then
        copy_lib "$QT_DIR/lib/$lib" "$OPT_DIR/lib"
    fi
done

log "Copy ICU libraries"

# Copy ICU libraries when they are included with Qt.
find "$QT_DIR/lib" -maxdepth 1 -type f \( \
    -name "libicui18n.so*" -o \
    -name "libicuuc.so*" -o \
    -name "libicudata.so*" \
\) -print | while read -r lib; do
    copy_lib "$lib" "$OPT_DIR/lib"
done

log "Copy Qt plugins"

for plugin_dir in \
    platforms \
    imageformats \
    iconengines \
    styles \
    generic \
    networkinformation \
    tls \
    sqldrivers \
    xcbglintegrations
do
    if [ -d "$QT_DIR/plugins/$plugin_dir" ]; then
        cp -a "$QT_DIR/plugins/$plugin_dir" "$OPT_DIR/plugins/"
        echo "Copied plugin dir: $plugin_dir"
    fi
done

log "Copy plugin dependencies"

find "$OPT_DIR/plugins" -type f -name "*.so" | while read -r plugin; do
    copy_ldd_deps "$plugin" "$OPT_DIR/lib"
done

log "Resolve recursive library dependencies"

# Scan copied libraries again for dependencies they introduce.
for round in 1 2 3; do
    echo "Dependency scan round: $round"
    find "$OPT_DIR/lib" -type f -name "*.so*" | while read -r so; do
        copy_ldd_deps "$so" "$OPT_DIR/lib"
    done
done

log "Patch RPATH"

patch_rpath "$OPT_DIR/$APP" '$ORIGIN/lib'
patch_rpath "$OPT_DIR/$CAPTURE_HELPER" '$ORIGIN/lib'

find "$OPT_DIR/lib" -type f -name "*.so*" | while read -r so; do
    patch_rpath "$so" '$ORIGIN'
done

find "$OPT_DIR/plugins" -type f -name "*.so" | while read -r so; do
    patch_rpath "$so" '$ORIGIN/../../lib:$ORIGIN/../lib:$ORIGIN'
done

log "Create launcher"

cat > "$DEBROOT/usr/bin/livoxviewerqt" <<EOF
#!/usr/bin/env bash
APP_DIR="/opt/$APP"

export LD_LIBRARY_PATH="\$APP_DIR/lib:\${LD_LIBRARY_PATH:-}"
export QT_PLUGIN_PATH="\$APP_DIR/plugins"
export QML2_IMPORT_PATH="\$APP_DIR/qml"
export QT_QPA_PLATFORM_PLUGIN_PATH="\$APP_DIR/plugins/platforms"

cd "\$APP_DIR"
exec "\$APP_DIR/$APP" "\$@"
EOF

chmod 755 "$DEBROOT/usr/bin/livoxviewerqt"

log "Create desktop file"

cat > "$DEBROOT/usr/share/applications/LivoxViewerQT.desktop" <<EOF
[Desktop Entry]
Type=Application
Name=LivoxViewerQT
Comment=Livox LiDAR Qt Viewer Application
Exec=livoxviewerqt
Icon=livoxviewerqt
Terminal=false
Categories=Utility;Graphics;
StartupWMClass=LivoxViewerQT
EOF

log "Create DEBIAN/control"

INSTALLED_SIZE="$(du -sk "$DEBROOT" | awk '{print $1}')"

cat > "$DEBROOT/DEBIAN/control" <<EOF
Package: $PKG_NAME
Version: $VERSION
Section: utils
Priority: optional
Architecture: $ARCH
Maintainer: FelixCooper1026
Installed-Size: $INSTALLED_SIZE
Depends: libc6, libstdc++6, libgcc-s1, libpcap0.8
Description: Livox LiDAR Qt Viewer Application
 LivoxViewerQT is a Qt/CMake based Livox LiDAR visualization and control tool.
 It includes Qt runtime libraries and can run without system Qt installation.
EOF

cat > "$DEBROOT/DEBIAN/postinst" <<'EOF'
#!/usr/bin/env bash
set -e

if command -v update-desktop-database >/dev/null 2>&1; then
    update-desktop-database /usr/share/applications || true
fi

if command -v gtk-update-icon-cache >/dev/null 2>&1; then
    gtk-update-icon-cache -f -t /usr/share/icons/hicolor || true
fi

exit 0
EOF

cat > "$DEBROOT/DEBIAN/postrm" <<'EOF'
#!/usr/bin/env bash
set -e

if command -v update-desktop-database >/dev/null 2>&1; then
    update-desktop-database /usr/share/applications || true
fi

if command -v gtk-update-icon-cache >/dev/null 2>&1; then
    gtk-update-icon-cache -f -t /usr/share/icons/hicolor || true
fi

exit 0
EOF

chmod 755 "$DEBROOT/DEBIAN/postinst"
chmod 755 "$DEBROOT/DEBIAN/postrm"

log "Validate dependencies"

echo "Executable dependency check:"
LD_LIBRARY_PATH="$OPT_DIR/lib:${LD_LIBRARY_PATH:-}" \
ldd "$OPT_DIR/$APP" | grep -E "Qt6|icu|not found" || true

echo
echo "Capture helper dependency check:"
LD_LIBRARY_PATH="$OPT_DIR/lib:${LD_LIBRARY_PATH:-}" \
ldd "$OPT_DIR/$CAPTURE_HELPER" | grep -E "pcap|not found" || true

if LD_LIBRARY_PATH="$OPT_DIR/lib:${LD_LIBRARY_PATH:-}" \
   ldd "$OPT_DIR/$APP" | grep -q "not found"; then
    echo
    echo "Missing dependencies:"
    LD_LIBRARY_PATH="$OPT_DIR/lib:${LD_LIBRARY_PATH:-}" ldd "$OPT_DIR/$APP"
    exit 1
fi

if LD_LIBRARY_PATH="$OPT_DIR/lib:${LD_LIBRARY_PATH:-}" \
   ldd "$OPT_DIR/$CAPTURE_HELPER" | grep -q "not found"; then
    echo
    echo "Missing capture helper dependencies:"
    LD_LIBRARY_PATH="$OPT_DIR/lib:${LD_LIBRARY_PATH:-}" ldd "$OPT_DIR/$CAPTURE_HELPER"
    exit 1
fi

echo
echo "Qt platform plugin dependency check:"
if [ -f "$OPT_DIR/plugins/platforms/libqxcb.so" ]; then
    LD_LIBRARY_PATH="$OPT_DIR/lib:${LD_LIBRARY_PATH:-}" \
    ldd "$OPT_DIR/plugins/platforms/libqxcb.so" | grep -E "not found|xcb|xkb|Qt6" || true

    if LD_LIBRARY_PATH="$OPT_DIR/lib:${LD_LIBRARY_PATH:-}" \
       ldd "$OPT_DIR/plugins/platforms/libqxcb.so" | grep -q "not found"; then
        echo
        echo "Missing platform plugin dependencies:"
        LD_LIBRARY_PATH="$OPT_DIR/lib:${LD_LIBRARY_PATH:-}" ldd "$OPT_DIR/plugins/platforms/libqxcb.so"
        exit 1
    fi
else
    die "Missing Qt platform plugin: plugins/platforms/libqxcb.so"
fi

log "Build deb"

mkdir -p "$DIST_DIR"
rm -f "$DIST_DIR/$DEB_NAME"

dpkg-deb --build "$DEBROOT" "$DIST_DIR/$DEB_NAME"

log "Done"

ls -lh "$DIST_DIR/$DEB_NAME"

echo
echo "Install:"
echo "  sudo apt install \"$DIST_DIR/$DEB_NAME\""
echo
echo "Run:"
echo "  livoxviewerqt"
echo
echo "Check dependencies after install:"
echo "  LD_LIBRARY_PATH=/opt/$APP/lib ldd /opt/$APP/$APP | grep -E 'not found|Qt6|icu'"
echo
echo "Uninstall:"
echo "  sudo apt remove $PKG_NAME"
