FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates curl git tar xz-utils zstd file perl python3 \
        build-essential gcc-10 g++-10 cmake ninja-build pkg-config \
        libopencv-dev libpcap-dev libgl1-mesa-dev libegl1-mesa-dev \
        libglu1-mesa-dev libfuse2 libx11-dev libx11-xcb-dev libxext-dev \
        libxrender-dev libxi-dev libxcb1-dev libxcb-glx0-dev \
        libxcb-keysyms1-dev libxcb-image0-dev libxcb-shm0-dev \
        libxcb-icccm4-dev libxcb-sync-dev libxcb-xfixes0-dev libxcb-shape0-dev \
        libxcb-randr0-dev libxcb-render-util0-dev libxcb-xinerama0-dev \
        libxcb-xkb-dev libxcb-cursor-dev libxkbcommon-dev libxkbcommon-x11-dev \
        libfontconfig1-dev libfreetype6-dev libssl-dev libsqlite3-dev \
        libdbus-1-dev libudev-dev patchelf desktop-file-utils \
    && rm -rf /var/lib/apt/lists/*

ENV QT_DIR=/opt/livox/qt/6.8.3 \
    LIVOX_THIRD_PARTY_ROOT=/opt/livox/third-party \
    CC=gcc-10 CXX=g++-10 BUILD_JOBS=2 QT_BUILD_JOBS=2 BOOST_VARIANTS=release \
    APPIMAGE_EXTRACT_AND_RUN=1 \
    APPIMAGETOOL=/usr/local/bin/appimagetool \
    APPIMAGE_RUNTIME_FILE=/opt/livox/tools/runtime-x86_64
ENV PATH="${QT_DIR}/bin:${PATH}"
ENV LD_LIBRARY_PATH="${QT_DIR}/lib"

COPY scripts/build_qt_linux.sh scripts/setup_third_party.sh /opt/livox/scripts/
RUN bash /opt/livox/scripts/build_qt_linux.sh
RUN bash /opt/livox/scripts/setup_third_party.sh \
    && rm -rf /opt/livox/third-party/.downloads /opt/livox/third-party/.build \
        /opt/livox/third-party/boost-1.82.0/bin.v2 /opt/livox/third-party/gtsam-4.2.0
RUN curl -fL --retry 3 \
        https://github.com/probonopd/linuxdeployqt/releases/download/continuous/linuxdeployqt-continuous-x86_64.AppImage \
        -o /usr/local/bin/linuxdeployqt \
    && curl -fL --retry 3 \
        https://github.com/AppImage/AppImageKit/releases/download/continuous/appimagetool-x86_64.AppImage \
        -o /usr/local/bin/appimagetool \
    && mkdir -p /opt/livox/tools \
    && curl -fL --retry 3 \
        https://github.com/AppImage/type2-runtime/releases/download/continuous/runtime-x86_64 \
        -o "$APPIMAGE_RUNTIME_FILE" \
    && chmod +x /usr/local/bin/linuxdeployqt /usr/local/bin/appimagetool
