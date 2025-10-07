#!/usr/bin/env python3
"""
PiDroponics MQTT Controller

Features:
- 8 relay outputs exposed as MQTT/HA switches:
  [fans, water_pump, air_pump, water_heater, led_strips, dosing_pump_1, dosing_pump_2, dosing_pump_3]
- Fan speed selector via PWM (0–100%)
- LED strip brightness selector via PWM (0–100%)
- Water temperature via DS18B20 (1-Wire)
- Air temperature & humidity via AHT10 (I2C)
- RPi system sensors: CPU temp, CPU %, mem %, disk %, load avg, uptime
- Home Assistant MQTT Discovery support

Environment variables (see /etc/default/pidroponics created by setup.sh):
  MQTT_BROKER_HOST, MQTT_BROKER_PORT, MQTT_USERNAME, MQTT_PASSWORD
  MQTT_DISCOVERY_PREFIX (default: homeassistant)
  MQTT_BASE_TOPIC (default: pidroponics)
  RELAY_ACTIVE_LOW (true/false; default true)
  RELAY_PINS (format "name:pin,name:pin,...")
  FAN_PWM_PIN, FAN_PWM_FREQ
  LED_PWM_PIN, LED_PWM_FREQ
  SENSOR_INTERVAL (seconds; default 30)

Dependencies:
  paho-mqtt, RPi.GPIO, smbus2, psutil
"""

import os
import time
import json
import socket
import signal
import threading
from pathlib import Path
from typing import Dict, Optional, Tuple, List

import psutil
import RPi.GPIO as GPIO
from smbus2 import SMBus, i2c_msg
import paho.mqtt.client as mqtt

# --------------------- Configuration ---------------------

def getenv_bool(name: str, default: bool) -> bool:
    val = os.getenv(name)
    if val is None:
        return default
    return str(val).strip().lower() in ("1", "true", "yes", "y", "on")

def getenv_int(name: str, default: int) -> int:
    val = os.getenv(name)
    if val is None or val.strip() == "":
        return default
    try:
        return int(val.strip())
    except ValueError:
        return default

HOSTNAME = socket.gethostname()
CLIENT_ID = f"pidroponics-{HOSTNAME}"

MQTT_BROKER_HOST = os.getenv("MQTT_BROKER_HOST", "localhost")
MQTT_BROKER_PORT = getenv_int("MQTT_BROKER_PORT", 1883)
MQTT_USERNAME = os.getenv("MQTT_USERNAME", "jamesmathewsprk@gmail.com")
MQTT_PASSWORD = os.getenv("MQTT_PASSWORD", "")

DISCOVERY_PREFIX = os.getenv("MQTT_DISCOVERY_PREFIX", "homeassistant").strip().strip("/")
BASE_TOPIC = os.getenv("MQTT_BASE_TOPIC", "pidroponics").strip().strip("/")

RELAY_ACTIVE_LOW = getenv_bool("RELAY_ACTIVE_LOW", True)

# Relay pin mapping (BCM). Default pins if env not provided.
DEFAULT_RELAYS = {
    "fans": 5,
    "water_pump": 6,
    "air_pump": 12,
    "water_heater": 16,
    "led_strips": 20,
    "dosing_pump_1": 21,
    "dosing_pump_2": 19,
    "dosing_pump_3": 26,
}

def parse_relays_env() -> Dict[str, int]:
    env = os.getenv("RELAY_PINS", "")
    if not env:
        return DEFAULT_RELAYS.copy()
    out: Dict[str, int] = {}
    pairs = [p for p in env.split(",") if p.strip()]
    for pair in pairs:
        if ":" not in pair:
            continue
        name, pin = pair.split(":", 1)
        name = name.strip()
        try:
            pin_num = int(pin.strip())
        except ValueError:
            continue
        if name:
            out[name] = pin_num
    # Fill any missing defaults to maintain expected names
    for k, v in DEFAULT_RELAYS.items():
        out.setdefault(k, v)
    return out

RELAYS = parse_relays_env()

FAN_PWM_PIN = getenv_int("FAN_PWM_PIN", 18)
FAN_PWM_FREQ = getenv_int("FAN_PWM_FREQ", 1000)
LED_PWM_PIN = getenv_int("LED_PWM_PIN", 13)
LED_PWM_FREQ = getenv_int("LED_PWM_FREQ", 1000)

SENSOR_INTERVAL = getenv_int("SENSOR_INTERVAL", 30)

# Device metadata for Home Assistant
DEVICE_INFO = {
    "identifiers": [f"pidroponics_{HOSTNAME}"],
    "manufacturer": "PiDroponics",
    "model": "Raspberry Pi Zero 2 W",
    "name": "PiDroponics Controller",
    "sw_version": "1.0.0",
}

AVAIL_TOPIC = f"{BASE_TOPIC}/status"

# --------------------- GPIO Setup ---------------------

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Relay outputs
for name, pin in RELAYS.items():
    GPIO.setup(pin, GPIO.OUT, initial=GPIO.HIGH if RELAY_ACTIVE_LOW else GPIO.LOW)

# PWM outputs
GPIO.setup(FAN_PWM_PIN, GPIO.OUT)
GPIO.setup(LED_PWM_PIN, GPIO.OUT)
fan_pwm = GPIO.PWM(FAN_PWM_PIN, FAN_PWM_FREQ)
led_pwm = GPIO.PWM(LED_PWM_PIN, LED_PWM_FREQ)
fan_pwm.start(0.0)  # percent duty cycle
led_pwm.start(0.0)

relay_states: Dict[str, bool] = {name: False for name in RELAYS}
fan_speed_pct: float = 0.0
led_brightness_pct: float = 0.0

def set_relay(name: str, on: bool):
    pin = RELAYS[name]
    if RELAY_ACTIVE_LOW:
        GPIO.output(pin, GPIO.LOW if on else GPIO.HIGH)
    else:
        GPIO.output(pin, GPIO.HIGH if on else GPIO.LOW)
    relay_states[name] = on

def set_pwm(fan_pct: Optional[float] = None, led_pct: Optional[float] = None):
    global fan_speed_pct, led_brightness_pct
    if fan_pct is not None:
        fan_speed_pct = max(0.0, min(100.0, float(fan_pct)))
        fan_pwm.ChangeDutyCycle(fan_speed_pct)
    if led_pct is not None:
        led_brightness_pct = max(0.0, min(100.0, float(led_pct)))
        led_pwm.ChangeDutyCycle(led_brightness_pct)

# --------------------- Sensors ---------------------

# AHT10 I2C sensor
class AHT10:
    ADDR = 0x38

    def __init__(self, busnum: int = 1):
        self.busnum = busnum
        self.bus: Optional[SMBus] = None
        self.available = False
        self._init_sensor()

    def _init_sensor(self):
        try:
            self.bus = SMBus(self.busnum)
            # Soft reset then init
            self.bus.write_byte(self.ADDR, 0xBA)  # soft reset
            time.sleep(0.02)
            self.bus.write_i2c_block_data(self.ADDR, 0xBE, [0x08, 0x00])  # init
            time.sleep(0.01)
            self.available = True
        except Exception:
            self.available = False
            try:
                if self.bus:
                    self.bus.close()
            except Exception:
                pass
            self.bus = None

    def read(self) -> Optional[Tuple[float, float]]:
        if not self.available or self.bus is None:
            return None
        try:
            # Trigger measurement
            self.bus.write_i2c_block_data(self.ADDR, 0xAC, [0x33, 0x00])
            time.sleep(0.08)  # typical measurement time
            # Read 6 bytes
            read = i2c_msg.read(self.ADDR, 6)
            self.bus.i2c_rdwr(read)
            data = list(read)
            if len(data) != 6:
                return None
            # Parse per datasheet
            # data[0]: status; [1..5]: payload
            hum_raw = ((data[1] << 12) | (data[2] << 4) | (data[3] >> 4)) & 0xFFFFF
            tmp_raw = (((data[3] & 0x0F) << 16) | (data[4] << 8) | data[5]) & 0xFFFFF

            humidity = (hum_raw / 1048576.0) * 100.0
            temperature = (tmp_raw / 1048576.0) * 200.0 - 50.0
            if humidity < 0 or humidity > 100:
                return None
            return (round(temperature, 2), round(humidity, 2))
        except Exception:
            return None

# DS18B20 1-Wire water temperature sensor
class DS18B20:
    def __init__(self):
        self.device_paths: List[Path] = []
        self._discover()

    def _discover(self):
        base = Path("/sys/bus/w1/devices")
        if not base.exists():
            # try to load modules (may already be loaded)
            os.system("modprobe w1-gpio >/dev/null 2>&1")
            os.system("modprobe w1-therm >/dev/null 2>&1")
        if base.exists():
            self.device_paths = list(base.glob("28-*"))

    def read_c(self) -> Optional[float]:
        if not self.device_paths:
            self._discover()
            if not self.device_paths:
                return None
        # Read first sensor found
        dev = self.device_paths[0] / "w1_slave"
        try:
            with dev.open("r") as f:
                lines = f.read().strip().splitlines()
            if len(lines) < 2 or "YES" not in lines[0]:
                return None
            # t=xxxxx in millidegrees C
            if "t=" not in lines[1]:
                return None
            t_str = lines[1].split("t=", 1)[1]
            t_c = float(t_str) / 1000.0
            return round(t_c, 2)
        except Exception:
            return None

# System sensors
def read_cpu_temp_c() -> Optional[float]:
    # Try psutil first
    try:
        temps = psutil.sensors_temperatures(fahrenheit=False)
        for key in ("cpu-thermal", "soc_thermal", "coretemp"):
            if key in temps and temps[key]:
                return round(temps[key][0].current, 2)
    except Exception:
        pass
    # Fallback to sysfs
    try:
        with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
            val = int(f.read().strip())
        return round(val / 1000.0, 2)
    except Exception:
        return None

def read_loadavg() -> Tuple[float, float, float]:
    try:
        return os.getloadavg()
    except Exception:
        return (0.0, 0.0, 0.0)

def read_uptime_s() -> int:
    try:
        with open("/proc/uptime", "r") as f:
            s = float(f.read().split()[0])
        return int(s)
    except Exception:
        return 0

# --------------------- MQTT Topics & Discovery ---------------------

def switch_topics(name: str) -> Tuple[str, str]:
    # return (cmd_topic, state_topic)
    return (f"{BASE_TOPIC}/switch/{name}/set",
            f"{BASE_TOPIC}/switch/{name}/state")

FAN_SPEED_CMD = f"{BASE_TOPIC}/fan/speed/set"
FAN_SPEED_STATE = f"{BASE_TOPIC}/fan/speed/state"
LED_BRIGHT_CMD = f"{BASE_TOPIC}/light/brightness/set"
LED_BRIGHT_STATE = f"{BASE_TOPIC}/light/brightness/state"

SENSOR_TOPICS = {
    "water_temp_c": f"{BASE_TOPIC}/sensor/water_temp_c",
    "aht10_temp_c": f"{BASE_TOPIC}/sensor/aht10_temp_c",
    "aht10_humidity_pct": f"{BASE_TOPIC}/sensor/aht10_humidity_pct",
    "cpu_temp_c": f"{BASE_TOPIC}/sensor/cpu_temp_c",
    "cpu_percent": f"{BASE_TOPIC}/sensor/cpu_percent",
    "mem_percent": f"{BASE_TOPIC}/sensor/mem_percent",
    "disk_percent": f"{BASE_TOPIC}/sensor/disk_percent",
    "load_1m": f"{BASE_TOPIC}/sensor/load_1m",
    "load_5m": f"{BASE_TOPIC}/sensor/load_5m",
    "load_15m": f"{BASE_TOPIC}/sensor/load_15m",
    "uptime_s": f"{BASE_TOPIC}/sensor/uptime_s",
}

def publish_discovery(client: mqtt.Client):
    # Switches
    for name in RELAYS:
        uid = f"{CLIENT_ID}_{name}"
        cmd_topic, state_topic = switch_topics(name)
        cfg_topic = f"{DISCOVERY_PREFIX}/switch/{CLIENT_ID}/{name}/config"
        payload = {
            "name": f"{name.replace('_',' ').title()}",
            "unique_id": uid,
            "command_topic": cmd_topic,
            "state_topic": state_topic,
            "payload_on": "ON",
            "payload_off": "OFF",
            "availability_topic": AVAIL_TOPIC,
            "device": DEVICE_INFO,
        }
        client.publish(cfg_topic, json.dumps(payload), retain=True, qos=1)

    # Fan speed selector (Number entity 0-100%)
    fan_cfg_topic = f"{DISCOVERY_PREFIX}/number/{CLIENT_ID}/fan_speed/config"
    fan_cfg = {
        "name": "Fan Speed",
        "unique_id": f"{CLIENT_ID}_fan_speed",
        "command_topic": FAN_SPEED_CMD,
        "state_topic": FAN_SPEED_STATE,
        "min": 0,
        "max": 100,
        "step": 1,
        "unit_of_measurement": "%",
        "icon": "mdi:fan",
        "availability_topic": AVAIL_TOPIC,
        "device": DEVICE_INFO,
    }
    client.publish(fan_cfg_topic, json.dumps(fan_cfg), retain=True, qos=1)

    # LED brightness selector (Number entity 0-100%)
    led_cfg_topic = f"{DISCOVERY_PREFIX}/number/{CLIENT_ID}/led_brightness/config"
    led_cfg = {
        "name": "LED Brightness",
        "unique_id": f"{CLIENT_ID}_led_brightness",
        "command_topic": LED_BRIGHT_CMD,
        "state_topic": LED_BRIGHT_STATE,
        "min": 0,
        "max": 100,
        "step": 1,
        "unit_of_measurement": "%",
        "icon": "mdi:led-strip-variant",
        "availability_topic": AVAIL_TOPIC,
        "device": DEVICE_INFO,
    }
    client.publish(led_cfg_topic, json.dumps(led_cfg), retain=True, qos=1)

    # Sensors - Home Assistant discovery for each
    # helper to build sensor discovery
    def sensor_cfg(key, name, device_class=None, unit=None, state_class="measurement", icon=None):
        return {
            "name": name,
            "unique_id": f"{CLIENT_ID}_{key}",
            "state_topic": SENSOR_TOPICS[key],
            "availability_topic": AVAIL_TOPIC,
            "device": DEVICE_INFO,
            **({"device_class": device_class} if device_class else {}),
            **({"unit_of_measurement": unit} if unit else {}),
            **({"state_class": state_class} if state_class else {}),
            **({"icon": icon} if icon else {}),
        }

    sensors = {
        "water_temp_c":  sensor_cfg("water_temp_c", "Water Temperature", "temperature", "°C"),
        "aht10_temp_c":  sensor_cfg("aht10_temp_c", "Air Temperature", "temperature", "°C"),
        "aht10_humidity_pct": sensor_cfg("aht10_humidity_pct", "Air Humidity", "humidity", "%"),
        "cpu_temp_c":    sensor_cfg("cpu_temp_c", "CPU Temperature", "temperature", "°C"),
        "cpu_percent":   sensor_cfg("cpu_percent", "CPU Usage", None, "%", icon="mdi:cpu-64-bit"),
        "mem_percent":   sensor_cfg("mem_percent", "Memory Usage", None, "%", icon="mdi:memory"),
        "disk_percent":  sensor_cfg("disk_percent", "Disk Usage", None, "%", icon="mdi:harddisk"),
        "load_1m":       sensor_cfg("load_1m", "Load 1m", None, None, icon="mdi:chart-line"),
        "load_5m":       sensor_cfg("load_5m", "Load 5m", None, None, icon="mdi:chart-line"),
        "load_15m":      sensor_cfg("load_15m", "Load 15m", None, None, icon="mdi:chart-line"),
        "uptime_s":      sensor_cfg("uptime_s", "Uptime", None, "s", state_class="measurement", icon="mdi:timer-outline"),
    }

    for key, cfg in sensors.items():
        topic = f"{DISCOVERY_PREFIX}/sensor/{CLIENT_ID}/{key}/config"
        client.publish(topic, json.dumps(cfg), retain=True, qos=1)

# --------------------- MQTT Client ---------------------

mqtt_client = mqtt.Client(client_id=CLIENT_ID, clean_session=True)
if MQTT_USERNAME:
    mqtt_client.username_pw_set(MQTT_USERNAME, password=MQTT_PASSWORD or None)

mqtt_client.will_set(AVAIL_TOPIC, payload="offline", qos=1, retain=True)

# Sensor devices
aht10 = AHT10(busnum=1)
ds18b20 = DS18B20()

# Synchronization
stop_event = threading.Event()

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        # Online
        client.publish(AVAIL_TOPIC, "online", retain=True, qos=1)
        # Publish discovery
        publish_discovery(client)
        # Subscribe to command topics
        subs = []
        for name in RELAYS:
            cmd_topic, _ = switch_topics(name)
            subs.append((cmd_topic, 1))
        subs.append((FAN_SPEED_CMD, 1))
        subs.append((LED_BRIGHT_CMD, 1))
        if subs:
            client.subscribe(subs)
        # Publish initial states
        publish_all_states(client)
    else:
        print(f"MQTT connect failed with code {rc}")

def on_message(client, userdata, msg):
    topic = msg.topic
    payload = (msg.payload or b"").decode("utf-8").strip()
    try:
        # Relay switch commands
        for name in RELAYS:
            cmd_topic, state_topic = switch_topics(name)
            if topic == cmd_topic:
                state = payload.upper() in ("1", "ON", "TRUE", "YES")
                set_relay(name, state)
                client.publish(state_topic, "ON" if relay_states[name] else "OFF", retain=True, qos=1)
                return

        # Fan speed (0-100)
        if topic == FAN_SPEED_CMD:
            try:
                value = float(payload)
            except ValueError:
                return
            set_pwm(fan_pct=value)
            client.publish(FAN_SPEED_STATE, f"{fan_speed_pct:.0f}", retain=True, qos=1)
            return

        # LED brightness (0-100)
        if topic == LED_BRIGHT_CMD:
            try:
                value = float(payload)
            except ValueError:
                return
            set_pwm(led_pct=value)
            client.publish(LED_BRIGHT_STATE, f"{led_brightness_pct:.0f}", retain=True, qos=1)
            return

    except Exception as e:
        print(f"Error handling message on {topic}: {e}")

def publish_all_states(client: mqtt.Client):
    # Relay states
    for name in RELAYS:
        _, state_topic = switch_topics(name)
        client.publish(state_topic, "ON" if relay_states[name] else "OFF", retain=True, qos=1)
    # PWM states
    client.publish(FAN_SPEED_STATE, f"{fan_speed_pct:.0f}", retain=True, qos=1)
    client.publish(LED_BRIGHT_STATE, f"{led_brightness_pct:.0f}", retain=True, qos=1)

def sensor_loop(client: mqtt.Client):
    last_pub = 0.0
    while not stop_event.is_set():
        now = time.time()
        if now - last_pub >= max(5, SENSOR_INTERVAL):
            last_pub = now
            try:
                # DS18B20 water temperature
                water_c = ds18b20.read_c()
                if water_c is not None:
                    client.publish(SENSOR_TOPICS["water_temp_c"], f"{water_c:.2f}", retain=False, qos=0)

                # AHT10 air temp/humidity
                if aht10.available:
                    res = aht10.read()
                    if res is not None:
                        air_temp_c, air_hum = res
                        client.publish(SENSOR_TOPICS["aht10_temp_c"], f"{air_temp_c:.2f}", retain=False, qos=0)
                        client.publish(SENSOR_TOPICS["aht10_humidity_pct"], f"{air_hum:.2f}", retain=False, qos=0)

                # System sensors
                cpu_temp_c = read_cpu_temp_c()
                if cpu_temp_c is not None:
                    client.publish(SENSOR_TOPICS["cpu_temp_c"], f"{cpu_temp_c:.2f}", retain=False, qos=0)

                cpu_percent = psutil.cpu_percent(interval=None)
                mem_percent = psutil.virtual_memory().percent
                disk_percent = psutil.disk_usage("/").percent
                l1, l5, l15 = read_loadavg()
                uptime_s = read_uptime_s()

                client.publish(SENSOR_TOPICS["cpu_percent"], f"{cpu_percent:.1f}", retain=False, qos=0)
                client.publish(SENSOR_TOPICS["mem_percent"], f"{mem_percent:.1f}", retain=False, qos=0)
                client.publish(SENSOR_TOPICS["disk_percent"], f"{disk_percent:.1f}", retain=False, qos=0)
                client.publish(SENSOR_TOPICS["load_1m"], f"{l1:.2f}", retain=False, qos=0)
                client.publish(SENSOR_TOPICS["load_5m"], f"{l5:.2f}", retain=False, qos=0)
                client.publish(SENSOR_TOPICS["load_15m"], f"{l15:.2f}", retain=False, qos=0)
                client.publish(SENSOR_TOPICS["uptime_s"], f"{uptime_s}", retain=False, qos=0)
            except Exception as e:
                print(f"Sensor publish error: {e}")
        stop_event.wait(1.0)

def cleanup():
    try:
        fan_pwm.stop()
        led_pwm.stop()
    except Exception:
        pass
    try:
        GPIO.cleanup()
    except Exception:
        pass

def handle_signals(signum, frame):
    stop_event.set()

def main():
    # Default all relays OFF at start
    for name in RELAYS:
        set_relay(name, False)

    # MQTT callbacks
    mqtt_client.on_connect = on_connect
    mqtt_client.on_message = on_message

    # Connect & start loop
    mqtt_client.connect(MQTT_BROKER_HOST, MQTT_BROKER_PORT, keepalive=60)
    mqtt_client.loop_start()

    # Publish availability on start
    mqtt_client.publish(AVAIL_TOPIC, "online", retain=True, qos=1)

    # Start sensor thread
    t = threading.Thread(target=sensor_loop, args=(mqtt_client,), daemon=True)
    t.start()

    # Handle signals for clean shutdown
    signal.signal(signal.SIGINT, handle_signals)
    signal.signal(signal.SIGTERM, handle_signals)

    # Wait for stop
    try:
        while not stop_event.is_set():
            time.sleep(0.5)
    finally:
        # Announce offline, cleanup, stop MQTT
        try:
            mqtt_client.publish(AVAIL_TOPIC, "offline", retain=True, qos=1)
        except Exception:
            pass
        mqtt_client.loop_stop()
        mqtt_client.disconnect()
        cleanup()

if __name__ == "__main__":
