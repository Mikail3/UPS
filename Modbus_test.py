import time
import configparser
import logging
from pymodbus.client import ModbusTcpClient
from pymodbus.exceptions import ModbusIOException

# Configure logging
logging.basicConfig(
    filename='modbus_errors.log',
    level=logging.INFO,
    format='%(asctime)s %(levelname)s: %(message)s'
)

# Load Modbus TCP settings
config = configparser.ConfigParser()
try:
    config.read('config.ini')
    ip = config.get('MODBUS', 'ip_address')
    port = config.getint('MODBUS', 'port')
    slave_id = config.getint('MODBUS', 'slave_id')
except (configparser.NoSectionError, configparser.NoOptionError) as e:
    print(f"Config file error: {e}")
    logging.error(f"Config file error: {e}")
    exit(1)

# Define registers
registers = {
    0x2000: ("Status Functions", 4, None, None),
    0x2002: ("Status Interface", 2, None, None),
    0x2006: ("Output Voltage", 1, "V", 1000),
    0x2007: ("Output Current", 1, "mA", None),
    0x200A: ("Battery Voltage", 1, "V", 1000),
    0x200B: ("Battery Current", 1, "mA", None),
    0x200D: ("Temperature", 1, "°C", 10),  # Verify scaling
    0x200E: ("Remaining Time", 1, "min", None),
    0x2052: ("Model Name", 10, "ASCII", None),
}

# Poll every 3 seconds
while True:
    try:
        client = ModbusTcpClient(ip, port=port, timeout=5)
        if not client.connect():
            print("Connection failed")
            logging.error(f"Connection to {ip}:{port} failed")
            time.sleep(3)
            continue

        logging.info(f"Connected to {ip}:{port}")
        print(f"\n--- Reading from {ip}:{port} (Slave ID {slave_id}) ---")

        for address, (label, word_count, unit, scale) in registers.items():
            result = client.read_holding_registers(address, word_count, slave=slave_id)

            if result is None or isinstance(result, ModbusIOException) or result.isError():
                print(f"{label:<20}: ERROR")
                logging.error(f"Failed to read {label} at 0x{address:04X}")
                continue

            values = result.registers
            logging.info(f"Read {label}: {' '.join(f'{v:04X}' for v in values)}")

            if label == "Model Name":
                if all(word == 0xFFFF for word in values):
                    print(f"{label:<20}: Unavailable")
                else:
                    try:
                        name = ''.join(chr((word >> 8) & 0xFF) + chr(word & 0xFF) for word in values).strip('\x00')
                        print(f"{label:<20}: {name}")
                    except UnicodeDecodeError:
                        print(f"{label:<20}: Invalid encoding (Raw: {' '.join(f'{v:04X}' for v in values)})")
            elif word_count == 1:
                raw = values[0]
                if raw == 0xFFFF:
                    print(f"{label:<20}: Unavailable (Raw: 0x{raw:04X})")
                else:
                    value = raw / scale if scale else raw
                    print(f"{label:<20}: {value:.2f} {unit or ''} (Raw: 0x{raw:04X})")
                    logging.info(f"{label}: {value:.2f} {unit or ''} (Raw: 0x{raw:04X})")
            else:
                if all(word == 0xFFFF for word in values):
                    print(f"{label:<20}: Unavailable")
                else:
                    hex_values = ' '.join(f"{v:04X}" for v in values)
                    print(f"{label:<20}: {hex_values}")

        client.close()

    except Exception as e:
        print(f"Unexpected error: {e}")
        logging.exception("Unexpected error occurred")

    time.sleep(3)