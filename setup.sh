#!/usr/bin/env bash
# PiDroponics - Raspberry Pi Zero 2 WH bootstrap script
# Installs system dependencies, clones/updates repo, creates Python venv,
# installs Python packages, configures a systemd service to start on boot, and starts it.
# Usage: curl -fsSL https://raw.githubusercontent.com/JamesCameronMathews/PiDroponics/main/setup.sh | bash
# Or run locally: bash setup.sh

set -euo pipefail

REPO_URL_DEFAULT="https://github.com/JamesCameronMathews/PiDroponics.git"
REPO_URL="${1:-$REPO_URL_DEFAULT}"
PROJECT_NAME="PiDroponics"
SERVICE_NAME="pidroponics"
SCRIPT_NAME="pidroponics_mqtt.py"

# Re-exec with sudo if not root (to allow apt/systemd changes)
if [[ "${EUID:-$(id -u)}" -ne 0 ]]; then
  echo "Elevating privileges with sudo..."
  exec sudo -E bash "$0" "$@"
fi

# Resolve the primary non-root user (prefer the user who invoked sudo)
RUN_USER="${SUDO_USER:-pi}"
RUN_GROUP="$(id -gn "$RUN_USER" 2>/dev/null || echo "$RUN_USER")"
USER_HOME="$(eval echo "~$RUN_USER")"
INSTALL_DIR="${USER_HOME}/${PROJECT_NAME}"

echo "Running as root. Target user: ${RUN_USER}"
echo "Install directory: ${INSTALL_DIR}"

# ---- Helpers ----
ensure_line_in_file() {
  local line="$1"
  local file="$2"
  mkdir -p "$(dirname "$file")"
  touch "$file"
  grep -qsF -- "$line" "$file" || echo "$line" >> "$file"
}

# ---- System packages ----
echo "Updating apt and installing dependencies..."
export DEBIAN_FRONTEND=noninteractive
apt-get update -y
apt-get install -y \
  git python3 python3-venv python3-pip python3-dev build-essential \
  libffi-dev libssl-dev \
  i2c-tools \
  bc \
  systemd

# ---- Enable I2C & 1-Wire (works for both Bullseye/Bookworm boot layouts) ----
BOOT_CONFIG=""
if [[ -f /boot/firmware/config.txt ]]; then
  BOOT_CONFIG="/boot/firmware/config.txt"
elif [[ -f /boot/config.txt ]]; then
  BOOT_CONFIG="/boot/config.txt"
fi

if [[ -n "$BOOT_CONFIG" ]]; then
  echo "Configuring interfaces in ${BOOT_CONFIG}..."
  ensure_line_in_file "dtparam=i2c_arm=on" "$BOOT_CONFIG"
  ensure_line_in_file "dtoverlay=w1-gpio" "$BOOT_CONFIG"
else
  echo "WARNING: Could not locate boot config file to enable I2C/1-Wire. Please enable manually."
fi

# Load modules at boot (older OSes)
ensure_line_in_file "i2c-dev" "/etc/modules"
ensure_line_in_file "w1-gpio" "/etc/modules"
ensure_line_in_file "w1-therm" "/etc/modules"

# Group membership for I2C access
adduser "$RUN_USER" i2c || true

# ---- Clone or update repo ----
if [[ -d "$INSTALL_DIR/.git" ]]; then
  echo "Repository exists, pulling latest..."
  sudo -u "$RUN_USER" git -C "$INSTALL_DIR" remote set-url origin "$REPO_URL" || true
  sudo -u "$RUN_USER" git -C "$INSTALL_DIR" fetch --all --prune
  sudo -u "$RUN_USER" git -C "$INSTALL_DIR" checkout main || true
  sudo -u "$RUN_USER" git -C "$INSTALL_DIR" pull --ff-only origin main || true
else
  echo "Cloning repository from: $REPO_URL"
  sudo -u "$RUN_USER" git clone "$REPO_URL" "$INSTALL_DIR"
fi

# ---- Python venv & requirements ----
echo "Creating/using Python virtual environment..."
sudo -u "$RUN_USER" bash -lc "python3 -m venv '$INSTALL_DIR/.venv'"
sudo -u "$RUN_USER" bash -lc "source '$INSTALL_DIR/.venv/bin/activate' && python -m pip install --upgrade pip wheel setuptools"

# Create requirements.txt if not present
REQ_FILE="$INSTALL_DIR/requirements.txt"
if [[ ! -f "$REQ_FILE" ]]; then
  echo "Generating default requirements.txt..."
  cat > "$REQ_FILE" <<'EOF'
paho-mqtt>=1.6.1
RPi.GPIO>=0.7.1
smbus2>=0.4.3
psutil>=5.9.0
EOF
  chown "$RUN_USER:$RUN_GROUP" "$REQ_FILE"
fi

echo "Installing Python packages from requirements.txt..."
sudo -u "$RUN_USER" bash -lc "source '$INSTALL_DIR/.venv/bin/activate' && pip install -r '$REQ_FILE'"

# ---- Environment file (for MQTT credentials & options) ----
ENV_FILE="/etc/default/${SERVICE_NAME}"
if [[ ! -f "$ENV_FILE" ]]; then
  echo "Creating $ENV_FILE (edit to set your MQTT details)..."
  cat > "$ENV_FILE" <<EOF
# Environment for ${SERVICE_NAME} service

# MQTT broker settings
MQTT_BROKER_HOST=localhost
MQTT_BROKER_PORT=1883
MQTT_USERNAME=jamesmathewsprk@gmail.com
MQTT_PASSWORD=

# Home Assistant discovery prefix
MQTT_DISCOVERY_PREFIX=homeassistant

# Base topic for this device
MQTT_BASE_TOPIC=pidroponics

# GPIO logic options
RELAY_ACTIVE_LOW=true

# GPIO pins (BCM numbering)
# 8 relays for: fans, water_pump, air_pump, water_heater, led_strips, dosing_pump_1, dosing_pump_2, dosing_pump_3
RELAY_PINS="fans:5,water_pump:6,air_pump:12,water_heater:16,led_strips:20,dosing_pump_1:21,dosing_pump_2:19,dosing_pump_3:26"

# PWM pins and frequency (Hz)
FAN_PWM_PIN=18
FAN_PWM_FREQ=1000
LED_PWM_PIN=13
LED_PWM_FREQ=1000

# Sensor publish interval (seconds)
SENSOR_INTERVAL=30
EOF
fi

# ---- systemd service ----
SERVICE_FILE="/etc/systemd/system/${SERVICE_NAME}.service"
if [[ ! -f "$SERVICE_FILE" ]]; then
  echo "Creating systemd service: $SERVICE_FILE"
  cat > "$SERVICE_FILE" <<EOF
[Unit]
Description=PiDroponics MQTT Controller
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=${RUN_USER}
Group=${RUN_GROUP}
EnvironmentFile=${ENV_FILE}
WorkingDirectory=${INSTALL_DIR}
ExecStart=${INSTALL_DIR}/.venv/bin/python ${INSTALL_DIR}/${SCRIPT_NAME}
Restart=always
RestartSec=3
# Ensure GPIO and I2C access
CapabilityBoundingSet=CAP_SYS_RAWIO
AmbientCapabilities=CAP_SYS_RAWIO

[Install]
WantedBy=multi-user.target
EOF
  systemctl daemon-reload
  systemctl enable "${SERVICE_NAME}.service"
fi

# ---- Start (or restart) service ----
echo "Starting ${SERVICE_NAME} service..."
systemctl restart "${SERVICE_NAME}.service"

echo
echo "Done!"
echo "Edit MQTT settings in: ${ENV_FILE}"
echo "Check service logs:    journalctl -u ${SERVICE_NAME} -f"
echo "Verify I2C devices:    sudo -u $RUN_USER i2cdetect -y 1 (AHT10 should be at 0x38)"
echo "Verify 1-Wire sensor:  ls /sys/bus/w1/devices/ (DS18B20 starts with 28-)"
