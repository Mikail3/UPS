import time
import os
import csv
import configparser
import logging
from pymodbus.client import ModbusTcpClient
from pymodbus.exceptions import ModbusIOException
from collections import deque
import json
import paho.mqtt.client as mqtt
import uuid
import socket

VALUES_LOG_FILE = "ups_values.csv"
ID_FILE = "ups_id.txt"
LOG_INTERVAL = 5  # seconds between CSV log writes

# Configure logging to file
logging.basicConfig(
    filename='modbus_errors.log',
    level=logging.INFO,
    format='%(asctime)s %(levelname)s: %(message)s'
)

# Define the Modbus registers we want to read: address -> (label, word count, unit, scale factor)
registers = {
    0x2000: ("Status Functions", 4, None, None),
    0x2002: ("Status Interface", 2, None, None),
    0x2006: ("Output Voltage", 1, "V", 1000),
    0x2007: ("Output Current", 1, "mA", 1),
    0x200A: ("Battery Voltage", 1, "V", 1000),
    0x200B: ("Battery Current", 1, "mA", 1),
    0x200D: ("Battery Temperature", 1, "K", 1),
    0x200E: ("Device Temperature", 1, "K", 1),
    0x203C: ("Remaining Time PC Shutdown (t31)", 1, "min", 60),
    0x2024: ("Battery Mode Time", 2, "min", 60),
    0x2026: ("User Battery Mode Time", 2, "min", 60),
    0x1064: ("Battery Capacity", 1, "100mAh", 10),
    0x0010: ("Device Name", 2, "ASCII", None),
}

def ensure_csv_header(filename):
    """Ensure CSV file has a header, create if missing."""
    header = [
        'Timestamp',
        'Battery Voltage (V)',
        'Output Current (mA)',
        'Battery Temp (°C)',
        'Device Temp (°C)',
        'Battery Current (mA)',
        'SOC (%)',
        'Battery Mode',
        'Battery Present',
        'Temp Sensor Connected',
        'MQTT Published'
    ]
    if not os.path.isfile(filename):
        with open(filename, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(header)
        logging.info(f"CSV header written to {filename}")

def publish_mqtt(client, topic, payload):
    """Publish JSON payload to MQTT topic with basic error handling."""
    try:
        result = client.publish(topic, payload)
        status = result.rc
        if status == 0:
            print(f"\nMQTT Published → Topic: {topic}")
            print(f"Payload: {payload}\n")
            logging.info(f"MQTT publish success: {payload}")
            return True
        else:
            print(f"MQTT Publish failed with status {status}")
            logging.error(f"MQTT publish failed with status {status}")
            return False
    except Exception as e:
        print(f"MQTT Publish exception: {e}")
        logging.error(f"MQTT publish exception: {e}")
        return False

def print_soc_graph(soc_history):
    """Print a simple ASCII graph showing State of Charge trend over last 10 minutes."""
    if not soc_history:
        print("SOC Trend: No data available")
        return

    now = time.time()
    window_start = now - 600  # last 10 minutes
    relevant = [s for t, s in soc_history if t >= window_start]
    if not relevant:
        print("SOC Trend: No recent data")
        return

    min_soc = min(relevant)
    max_soc = max(relevant)
    if max_soc - min_soc < 5:
        mid = (max_soc + min_soc) / 2
        min_soc = mid - 2.5
        max_soc = mid + 2.5

    cols = 50
    rows = 10
    chunk_size = max(1, len(relevant) // cols)
    avg_socs = [sum(relevant[i:i+chunk_size])/chunk_size for i in range(0, len(relevant), chunk_size)][:cols]

    print("\nSOC Trend (Last 10 minutes):")
    for r in range(rows, -1, -1):
        threshold = min_soc + (max_soc - min_soc) * (r / rows)
        line = f"{threshold:5.1f}% | "
        for soc in avg_socs:
            line += '█' if soc >= threshold else ' '
        print(line)
    print("      +" + "-" * cols)
    print("       " + ''.join(str(i//5) if i % 5 == 0 else ' ' for i in range(cols)))
    print()

def format_register_value(label, values, word_count, unit, scale):
    """Format the register values nicely as a string."""
    if all(v == 0xFFFF for v in values):
        return f"{label:<35}: Unavailable"
    if word_count == 1:
        raw = values[0]
        if raw == 0xFFFF:
            return f"{label:<35}: Unavailable (Raw: 0x{raw:04X})"
        value = raw / scale if scale else raw
        if unit == "K":  # Kelvin to Celsius conversion
            celsius = value - 273.15
            return f"{label:<35}: {celsius:.2f} °C (Raw: 0x{raw:04X})"
        elif unit == "ASCII":
            # ASCII for single word is unlikely, handled as 2-word
            return f"{label:<35}: {''.join(chr(raw))}"
        else:
            return f"{label:<35}: {value:.2f} {unit or ''} (Raw: 0x{raw:04X})"
    else:
        # For 2 or more words
        raw_value = (values[0] << 16) + values[1]
        value = raw_value / scale if scale else raw_value
        if unit == "ASCII":
            # Convert 2 words to ASCII string (4 chars)
            name = ''.join(chr((raw_value >> (8 * i)) & 0xFF) for i in reversed(range(4)))
            return f"{label:<35}: {name.strip(chr(0))}"
        else:
            return f"{label:<35}: {value:.2f} {unit or ''} (Raw: 0x{raw_value:08X})"

def read_ascii_string(client, start_addr, length, slave_id):
    """Read multiple registers and decode as ASCII string (2 chars per register)."""
    res = client.read_holding_registers(start_addr, length, slave=slave_id)
    if not res or res.isError() or all(v == 0xFFFF for v in res.registers):
        logging.warning(f"Failed to read ASCII string at 0x{start_addr:04X}")
        return None
    chars = []
    for reg in res.registers:
        chars.append(chr((reg >> 8) & 0xFF))
        chars.append(chr(reg & 0xFF))
    string_val = ''.join(chars).strip('\x00').strip()
    logging.info(f"Read ASCII string at 0x{start_addr:04X}: '{string_val}'")
    return string_val

def main():
    ensure_csv_header(VALUES_LOG_FILE)

    # Load configuration from config.ini
    config = configparser.ConfigParser()
    config.read('config.ini')

    ip = config.get('MODBUS', 'ip_address')
    port = config.getint('MODBUS', 'port')
    slave_id = config.getint('MODBUS', 'slave_id')

    mqtt_broker = config.get('MQTT', 'broker_address')
    mqtt_port = config.getint('MQTT', 'broker_port')
    mqtt_username = config.get('MQTT', 'username')
    mqtt_password = config.get('MQTT', 'password')
    mqtt_base_topic = config.get('MQTT', 'base_topic').rstrip('/')

    # Setup MQTT client and connect
    mqtt_client = mqtt.Client()
    mqtt_client.username_pw_set(mqtt_username, mqtt_password)
    try:
        mqtt_client.connect(mqtt_broker, mqtt_port)
        mqtt_client.loop_start()
        print(f"MQTT - Connected to broker {mqtt_broker}:{mqtt_port}")
        logging.info(f"Connected to MQTT broker {mqtt_broker}:{mqtt_port}")
    except Exception as e:
        print(f"MQTT - Connection failed: {e}")
        logging.error(f"MQTT connection failed: {e}")

    soc_history = deque(maxlen=600)
    last_log_time = 0

    while True:
        client = ModbusTcpClient(ip, port=port, timeout=5)
        if not client.connect():
            print(f"MODBUS - Connection failed to {ip}:{port}")
            logging.error(f"Modbus connection failed to {ip}:{port}")
            time.sleep(5)
            continue
        else:
            print(f"\nMODBUS - Connected to {ip}:{port}")
            logging.info(f"Modbus connected to {ip}:{port}")

        device_name = None
        battery_mode = False
        mqtt_published = False

        try:
            # Read Device Name (0x0012) just to have it
            res_name = client.read_holding_registers(0x0012, 2, slave=slave_id)
            if res_name and not res_name.isError() and not all(v == 0xFFFF for v in res_name.registers):
                device_name = ''.join(
                    chr((res_name.registers[i] >> 8) & 0xFF) + chr(res_name.registers[i] & 0xFF)
                    for i in range(2)
                ).strip('\x00')
                logging.info(f"Device name read: {device_name}")
            else:
                device_name = "UnknownDevice"
                logging.warning("Device name unavailable")

            # Read Status Functions for battery mode & sensor status
            res_status = client.read_holding_registers(0x2000, 4, slave=slave_id)
            if res_status and not res_status.isError():
                regs = res_status.registers
                battery_mode = (regs[0] & 0x0004) != 0
                battery_present = (regs[1] & 0x0100) != 0
                temp_sensor_connected = (regs[1] & 0x1000) != 0
                logging.info(f"Status registers read: battery_mode={battery_mode}, battery_present={battery_present}, temp_sensor={temp_sensor_connected}")
            else:
                battery_present = False
                temp_sensor_connected = False
                logging.warning("Status registers unavailable")

            print(f"Battery Mode          : {'Active (Battery Mode)' if battery_mode else 'Inactive (Grid Power)'}")
            print(f"Battery Present       : {'Yes' if battery_present else 'No'}")
            print(f"Temperature Sensor    : {'Connected' if temp_sensor_connected else 'Not Connected'}")
            print(f"Device Name           : {device_name}")

            # Read all registers and print formatted values
            register_values = {}
            for addr, (label, word_count, unit, scale) in registers.items():
                res = client.read_holding_registers(addr, word_count, slave=slave_id)
                if not res or res.isError():
                    print(f"{label:<35}: ERROR reading")
                    logging.error(f"Error reading register {label} (0x{addr:04X})")
                    register_values[label] = None
                    continue
                reg_val_str = format_register_value(label, res.registers, word_count, unit, scale)
                print(reg_val_str)
                register_values[label] = res.registers

            # Calculate SOC based on Battery Voltage register
            battery_voltage = None
            soc = None
            if register_values.get("Battery Voltage"):
                raw_voltage = register_values["Battery Voltage"][0]
                if raw_voltage != 0xFFFF:
                    battery_voltage = raw_voltage / 1000
                    # Linear SOC estimate from voltage, clamp 0-100%
                    soc = 100 * (battery_voltage - 20.4) / (27.5 - 20.4)
                    soc = max(0, min(100, soc))
                    soc_history.append((time.time(), soc))
                    print(f"\nEstimated Battery SOC  : {soc:.2f}% (Voltage: {battery_voltage:.2f} V)")
                    logging.info(f"Calculated SOC: {soc:.2f}%")
                else:
                    print("\nEstimated Battery SOC  : N/A (Battery Voltage not available)")
                    logging.warning("Battery Voltage unavailable, cannot calculate SOC")

            # Print SOC trend graph
            print_soc_graph(soc_history)

            # Ensure persistent unique ID stored in ups_id.txt
            import uuid

            

            if os.path.exists(ID_FILE):
                with open(ID_FILE, 'r') as f:
                    unique_id = f.read().strip()
            else:
                unique_id = uuid.uuid4().hex[:8]
                with open(ID_FILE, 'w') as f:
                    f.write(unique_id)

            logging.info(f"Using Unique ID: {unique_id}")

            safe_unique_id = unique_id.replace(' ', '_').replace('/', '_')
            print(f"Unique ID             : {unique_id}")
            print(f"Safe Unique ID        : {safe_unique_id}")
            mqtt_topic = f"{mqtt_base_topic}/{safe_unique_id}"
            print(f"MQTT Topic            : {mqtt_topic}")


            payload_data = {
            "timestamp": time.strftime('%Y-%m-%d %H:%M:%S'),
            "battery_mode": battery_mode,
            "battery_present": battery_present,
            "soc_percent": f"{round(soc, 2)}%" if soc is not None else None,
            "battery_voltage": f"{battery_voltage:.2f} V" if battery_voltage is not None else None,
            "output_current": f"{register_values.get('Output Current')[0]} mA" if register_values.get("Output Current") else None,
            "battery_temperature": f"{(register_values.get('Battery Temperature')[0] - 273.15):.2f} °C" if register_values.get("Battery Temperature") and register_values.get("Battery Temperature")[0] != 0xFFFF else None
        }

            payload = json.dumps(payload_data)
            mqtt_published = publish_mqtt(mqtt_client, mqtt_topic, payload)

            # Log key data to CSV at intervals
            if time.time() - last_log_time > LOG_INTERVAL:
                last_log_time = time.time()
                with open(VALUES_LOG_FILE, mode='a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow([
                        time.strftime('%Y-%m-%d %H:%M:%S'),
                        round(battery_voltage, 3) if battery_voltage is not None else '',
                        (register_values.get("Output Current")[0] if register_values.get("Output Current") else ''),
                        (register_values.get("Battery Temperature")[0] - 273.15 if register_values.get("Battery Temperature") and register_values.get("Battery Temperature")[0] != 0xFFFF else ''),
                        (register_values.get("Device Temperature")[0] - 273.15 if register_values.get("Device Temperature") and register_values.get("Device Temperature")[0] != 0xFFFF else ''),
                        (register_values.get("Battery Current")[0] if register_values.get("Battery Current") else ''),
                        round(soc, 2) if soc is not None else '',
                        battery_mode,
                        battery_present,
                        temp_sensor_connected,
                        mqtt_published
                    ])
                print(f"Data logged to {VALUES_LOG_FILE}")
                logging.info("Data logged to CSV file")

        except Exception as e:
            print(f"Unexpected error: {e}")
            logging.exception("Unexpected error occurred")

        finally:
            client.close()
            logging.info("Modbus client connection closed")

        # Wait 5 seconds if battery mode active, else 10 seconds
        wait_time = 5 if battery_mode else 10
        print(f"\nWaiting {wait_time} seconds...\n{'='*60}")
        time.sleep(wait_time)

if __name__ == "__main__":
    main()
