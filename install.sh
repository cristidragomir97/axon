#!/bin/bash

# Installation script for Axon udev rules on Raspberry Pi

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "Installing Axon udev rules..."

# Copy enumeration script
sudo cp "$SCRIPT_DIR/axon-enumerate.sh" /usr/local/bin/
sudo chmod +x /usr/local/bin/axon-enumerate.sh
echo "✓ Installed enumeration script"

# Copy udev rules  
sudo cp "$SCRIPT_DIR/99-axon-devices.rules" /etc/udev/rules.d/
echo "✓ Installed udev rules"

# Reload udev
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty
echo "✓ Reloaded udev rules"

echo ""
echo "Installation complete!"
echo ""
echo "Connected Axon devices will appear as:"
echo "  /dev/axon-1-rs485"
echo "  /dev/axon-1-feetech" 
echo "  /dev/axon-1-dynamixel"
echo "  /dev/axon-1-uart0"
echo "  /dev/axon-1-uart1"
echo "  /dev/axon-2-rs485"
echo "  /dev/axon-2-feetech"
echo "  etc."
echo ""
echo "Device numbers are assigned consistently based on serial number ordering."