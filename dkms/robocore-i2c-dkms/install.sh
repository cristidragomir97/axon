#!/bin/bash
set -e

VERSION="1.0.0"
MODULE="i2c-robocore"
DKMS_DIR="/usr/src/${MODULE}-${VERSION}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

ok() {
    echo "[OK] $1"
}

fail() {
    echo "[FAIL] $1"
    exit 1
}

do_install() {
    echo "Installing RoboCore I2C driver v${VERSION}..."
    echo ""

    # Check prerequisites
    which dkms > /dev/null 2>&1 || fail "dkms not installed. Run: sudo apt install dkms"
    if [ ! -d "/lib/modules/$(uname -r)/build" ]; then
        fail "Kernel headers not installed. Run: sudo apt install linux-headers-$(uname -r)"
    fi

    # Check source files exist
    if [ ! -f "${SCRIPT_DIR}/i2c-robocore.c" ]; then
        fail "i2c-robocore.c not found in ${SCRIPT_DIR}"
    fi

    # Remove old version if present
    sudo dkms remove "${MODULE}/${VERSION}" --all 2>/dev/null || true
    sudo rm -rf "${DKMS_DIR}"

    # Copy source to DKMS directory
    sudo mkdir -p "${DKMS_DIR}"
    sudo cp "${SCRIPT_DIR}/i2c-robocore.c" "${DKMS_DIR}/"
    sudo cp "${SCRIPT_DIR}/Makefile" "${DKMS_DIR}/"
    sudo cp "${SCRIPT_DIR}/dkms.conf" "${DKMS_DIR}/"
    ok "Source installed to ${DKMS_DIR}"

    # Register with DKMS
    sudo dkms add "${MODULE}/${VERSION}"
    ok "DKMS module registered"

    # Build
    sudo dkms build "${MODULE}/${VERSION}"
    ok "Module built for kernel $(uname -r)"

    # Install
    sudo dkms install "${MODULE}/${VERSION}"
    ok "Module installed"

    # Install udev rules
    sudo cp "${SCRIPT_DIR}/99-robocore.rules" /etc/udev/rules.d/
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    ok "Udev rules installed"

    # Blacklist i2c-tiny-usb so it doesn't compete
    echo "blacklist i2c_tiny_usb" | sudo tee /etc/modprobe.d/robocore-blacklist.conf > /dev/null
    ok "Blacklisted i2c-tiny-usb (prevents conflict)"

    # Load it now
    sudo modprobe "${MODULE}" 2>/dev/null || true
    ok "Module loaded"

    echo ""
    echo "Done! Plug in your Link101."
    echo "I2C will appear at /dev/robocore/i2c"
    echo "Test with: i2cdetect -l | grep -i robocore"
}

do_uninstall() {
    echo "Removing RoboCore I2C driver..."
    echo ""

    sudo rmmod "${MODULE}" 2>/dev/null || true
    sudo dkms remove "${MODULE}/${VERSION}" --all 2>/dev/null || true
    sudo rm -rf "${DKMS_DIR}"
    sudo rm -f /etc/udev/rules.d/99-robocore.rules
    sudo rm -f /etc/modprobe.d/robocore-blacklist.conf
    sudo udevadm control --reload-rules

    ok "Driver removed"
}

case "${1}" in
    install)   do_install ;;
    uninstall) do_uninstall ;;
    *)
        echo "Usage: $0 {install|uninstall}"
        exit 1
        ;;
esac