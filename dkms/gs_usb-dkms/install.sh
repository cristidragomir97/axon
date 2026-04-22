#!/bin/bash

# RoboCore Axon gs_usb DKMS installer
# Installs gs_usb kernel module with support for RoboCore Axon CAN interface

set -e

PACKAGE_NAME="gs_usb"
PACKAGE_VERSION="1.0"
PACKAGE_DIR="/usr/src/${PACKAGE_NAME}-${PACKAGE_VERSION}"

echo "Installing RoboCore Axon gs_usb DKMS module..."

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo "Please run as root (use sudo)"
    exit 1
fi

# Install required packages
echo "Installing dependencies..."
apt-get update -qq
apt-get install -y dkms linux-headers-$(uname -r) || {
    echo "Warning: Could not install linux-headers. Make sure kernel headers are available."
}

# Create package directory
echo "Setting up DKMS package..."
mkdir -p "$PACKAGE_DIR"
cp -r * "$PACKAGE_DIR/"

# Add to DKMS
echo "Adding module to DKMS..."
dkms add -m "$PACKAGE_NAME" -v "$PACKAGE_VERSION" || {
    echo "Module already exists in DKMS, removing old version..."
    dkms remove -m "$PACKAGE_NAME" -v "$PACKAGE_VERSION" --all || true
    dkms add -m "$PACKAGE_NAME" -v "$PACKAGE_VERSION"
}

# Build module
echo "Building module..."
dkms build -m "$PACKAGE_NAME" -v "$PACKAGE_VERSION"

# Install module
echo "Installing module..."
dkms install -m "$PACKAGE_NAME" -v "$PACKAGE_VERSION"

# Load CAN modules
echo "Loading CAN modules..."
modprobe can
modprobe can-dev
modprobe gs_usb

echo "Installation complete!"
echo ""
echo "The gs_usb module now supports RoboCore Axon devices (VID:1209 PID:AC01)"
echo "Connect your Axon device and check 'dmesg' for CAN interface creation."
echo "Use 'ip link show' to see available CAN interfaces."
echo ""
echo "To configure CAN interface:"
echo "  sudo ip link set can0 type can bitrate 500000"
echo "  sudo ip link set can0 up"
echo ""
echo "To uninstall:"
echo "  sudo dkms remove gs_usb/1.0 --all"