#!/bin/bash
set -e

RULE_FILE="/etc/udev/rules.d/90-nhatbot.rules"

backup_if_exists() {
  if [ -f "$RULE_FILE" ]; then
    ts=$(date +%Y%m%d_%H%M%S)
    sudo cp -a "$RULE_FILE" "${RULE_FILE}.bak_${ts}"
    echo "↪️  Backed up: ${RULE_FILE}.bak_${ts}"
  fi
}

write_rules() {
  echo "📝 Writing $RULE_FILE ..."
  sudo tee "$RULE_FILE" > /dev/null <<'EOF'
# RPLIDAR (CP210x, serial 0001) -> /dev/rplidar
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="0001", \
  MODE:="0660", GROUP="plugdev", SYMLINK+="rplidar"

# ZLAC-8015D (FTDI, serial AB0PJ5HV) -> /dev/zlac_8015d
KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTRS{serial}=="AB0PJ5HV", \
  MODE:="0660", GROUP="plugdev", SYMLINK+="zlac_8015d"

# ESP (CP210x) cố định theo cổng vật lý (ví dụ 1-2.4) -> /dev/esp_device
# Đổi 1-2.4 cho khớp máy của bạn nếu khác.
KERNEL=="ttyUSB*", KERNELS=="1-2.4", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", \
  MODE:="0660", GROUP="dialout", SYMLINK+="esp_device"

# Arduino Mega (CDC ACM) -> /dev/arduino_mega
KERNEL=="ttyACM*", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0042", ATTRS{serial}=="55739323837351017271", \
  MODE:="0660", GROUP="dialout", SYMLINK+="arduino_mega"
EOF
}

reload_rules() {
  echo "🔁 Reloading udev rules..."
  sudo udevadm control --reload-rules
  # Kích hoạt cho subsystem tty
  sudo udevadm trigger -s tty || true
  echo "🔌 If aliases don't appear, replug the USB devices."
}

post_check_alias() {
  alias="$1"
  if [ -e "/dev/$alias" ]; then
    echo "✅ /dev/$alias exists:"
    ls -l "/dev/$alias"
    real=$(readlink -f "/dev/$alias" || true)
    [ -n "$real" ] && echo "   ↪️  real device: $real"
  else
    echo "⚠️  /dev/$alias not found (replug may be needed)."
  fi
}

ensure_groups() {
  # Bạn đã thuộc plugdev/dialout rồi, nhưng check nhẹ:
  if ! groups "$USER" | grep -q '\bplugdev\b'; then
    echo "➕ Adding $USER to plugdev ..."
    sudo usermod -aG plugdev "$USER"
    echo "   (logout/login to apply)"
  fi
  if ! groups "$USER" | grep -q '\bdialout\b'; then
    echo "➕ Adding $USER to dialout ..."
    sudo usermod -aG dialout "$USER"
    echo "   (logout/login to apply)"
  fi
}

main() {
  backup_if_exists
  write_rules
  ensure_groups
  reload_rules

  # In chẩn đoán nhanh
  echo "🔎 Post-check aliases:"
  post_check_alias "rplidar"
  post_check_alias "zlac_8015d"
  post_check_alias "esp_device"
  post_check_alias "arduino_mega"

  echo "✅ Done."
}

main "$@"
