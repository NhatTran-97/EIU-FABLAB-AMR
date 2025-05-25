#!/bin/bash

RULE_FILE="/etc/udev/rules.d/90-nhatbot.rules"

if [ ! -f "$RULE_FILE" ]; then
    echo "Creating $RULE_FILE..."
    sudo tee "$RULE_FILE" > /dev/null <<EOF
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="0001", MODE="0660", GROUP="plugdev", SYMLINK+="rplidar"
KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTRS{serial}=="AB0PJ5HV", MODE="0660", GROUP="plugdev", SYMLINK+="zlac_8015d"
EOF
else
    echo "$RULE_FILE already exists. Skipping creation."
fi

# Thêm user hiện tại vào plugdev nếu chưa có
if ! groups $USER | grep -q "\bplugdev\b"; then
    echo "Adding $USER to plugdev group..."
    sudo usermod -aG plugdev $USER
    echo "Please logout and login again to apply group changes."
else
    echo "$USER is already in plugdev group."
fi

# Reload udev rules
echo "Reloading udev rules..."
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "✅ Done. Please replug USB devices or reboot if needed."
