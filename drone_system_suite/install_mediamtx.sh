#!/bin/bash
# MediaMTX Auto-Installer Script for Raspberry Pi & Linux

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TOOLS_DIR="$SCRIPT_DIR/.tools/mediamtx"

mkdir -p "$TOOLS_DIR"

if [ -f "$TOOLS_DIR/mediamtx" ]; then
    echo "[MediaMTX] Binary already installed at: $TOOLS_DIR/mediamtx"
    exit 0
fi

ARCH=$(uname -m)
case "$ARCH" in
    aarch64|arm64)
        TAR_ARCH="linux_arm64v8"
        ;;
    armv7l|armhf)
        TAR_ARCH="linux_armv7"
        ;;
    x86_64)
        TAR_ARCH="linux_amd64"
        ;;
    *)
        echo "[MediaMTX ERROR] Unsupported architecture: $ARCH"
        exit 1
        ;;
esac

VERSION="v1.9.3"
URL="https://github.com/bluenviron/mediamtx/releases/download/${VERSION}/mediamtx_${VERSION}_${TAR_ARCH}.tar.gz"

echo "[MediaMTX] Downloading MediaMTX ${VERSION} (${TAR_ARCH}) from GitHub..."
if command -v wget >/dev/null 2>&1; then
    wget -qO- "$URL" | tar -xz -C "$TOOLS_DIR"
elif command -v curl >/dev/null 2>&1; then
    curl -sL "$URL" | tar -xz -C "$TOOLS_DIR"
else
    echo "[MediaMTX ERROR] Neither wget nor curl found on system."
    exit 1
fi

chmod +x "$TOOLS_DIR/mediamtx"
echo "[MediaMTX] Successfully installed to $TOOLS_DIR/mediamtx"
