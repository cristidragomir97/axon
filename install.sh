#!/bin/bash

# Installation script for Link101 udev rules on Raspberry Pi

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "Installing Link101 udev rules..."

sudo cp "$SCRIPT_DIR/99-link101-devices.rules" /etc/udev/rules.d/
echo "✓ Installed udev rules"

sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty
echo "✓ Reloaded udev rules"

echo ""
echo "Installation complete!"
echo ""
echo "Connected Link101 devices will appear as:"
echo "  /dev/link101-rs485"
echo "  /dev/link101-servo"
echo "  /dev/link101-serial1"
echo "  /dev/link101-serial2"
echo "  /dev/link101-serial3    (if UART2 enabled)"
echo ""
echo "Verify with: ls -la /dev/link101-*"