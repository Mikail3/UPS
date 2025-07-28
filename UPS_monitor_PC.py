"""
===============================================================================
 Script Name  : UPS_monitor_PC.py
 Description  : Monitors a Phoenix Contact TRIO UPS via Modbus TCP. ( Device is MODBUS RTU but we use a Modbus TCP to RTU gateway)
                Reads key registers such as status, battery voltage/current,
                temperature, and calculates an estimated State of Charge (SOC). Device does not support SOC register, so we estimate SOC
                based on battery voltage
                using linear interpolation. Displays trend graph of SOC for
                the last 10 minutes and logs errors to a file.

 Author       : MCI
 Created On   : 2025-07-23
 Last Updated : 2025-07-23
 Version      : 1.1
 Python       : 3.10+
 Dependencies : pymodbus, configparser, logging, collections
===============================================================================

 Features:
 - Connects to a UPS using Modbus TCP, the connection parameters are read from a config file ( config.ini )
 - Reads multiple holding registers including:
     - Status, Battery Voltage, Battery Current, Temperatures
 - Calculates SOC using voltage-based linear interpolation between:
     - 20.4V (0%) and 27.5V (100%)
 - Displays ASCII graph of SOC over last 10 minutes
 - Logs errors and raw register values
 - Detects if UPS is in battery or mains mode
 - Handles connection errors and retries
 - Displays device name if available
 - Handles missing or unavailable registers gracefully
    - Displays battery capacity in 100mAh units
    - Displays temperature in Celsius
    - Displays voltage in Volts, current in mA
    - Displays remaining time for PC shutdown in minutes
    - Displays user battery mode time in minutes (feauture barely works, due to the fact that the UPS does not support this register and it is not implemented in the device)
    - Displays battery mode time in minutes
    - Displays device temperature in Kelvin, converted to Celsius
    - Displays status of battery presence and temperature sensor connection
    - Handles Modbus exceptions and logs them


 Notes:
 - SOC estimation is voltage-based and not highly accurate under heavy load, 
   but provides a rough estimate.
 - Consider implementing coulomb counting for better SOC accuracy
===============================================================================
"""


import time
import configparser
import logging
from pymodbus.client import ModbusTcpClient
from pymodbus.exceptions import ModbusIOException
from collections import deque

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

SOC_HISTORY = deque(maxlen=600)

while True:
    try:
        client = ModbusTcpClient(ip, port=port, timeout=5)
        if not client.connect():
            print("Connection failed")
            logging.error(f"Connection to {ip}:{port} failed")
            time.sleep(3)
            continue

        print(f"\n--- Reading from {ip}:{port} (Slave ID {slave_id}) ---")
        logging.info(f"Connected to {ip}:{port} (Slave ID {slave_id})")

        # Calculate SOC using linear interpolation methods
        # Explanation:
        #   - We assume 0% SOC at 20.4V (typical discharge limit for 24V lead-acid), not sure if this is correct for our UPS but it seems to be the case
        #   - We assume 100% SOC at 27.5V (fully charged voltage) 
        #   - SOC is calculated linearly between these two voltage points
        #   Formula:
        #       SOC = 100 * (battery_voltage - V_min) / (V_max - V_min) 
        #   Where:
        #       V_min = 20.4V (0% SOC)
        #       V_max = 27.5V (100% SOC)
        #   Finally, clamp between 0% and 100%
        # If the battery voltage is below 20.4V, we assume SOC is 0%
        # If the battery voltage is above 27.5V, we assume SOC is 100%
        #      This is a simplification, but it should work for most lead-acid batteries used in UPS systems.
        # For the ATEX battery, we assume the same voltage limits apply.

        result = client.read_holding_registers(0x2000, 4, slave=slave_id)
        battery_mode = False
        if result and not isinstance(result, ModbusIOException) and not result.isError():
            status_functions = result.registers
            battery_mode = (status_functions[0] & 0x0004) != 0  # Bit 2 in first word
            battery_present = (status_functions[1] & 0x0100) != 0  # Bit 8 in tweede woord  # Bit 8 in first word
            temp_sensor_connected = (status_functions[1] & 0x1000) != 0  # Bit 28 in second word
            print(f"Battery Mode Status                : {'Active' if battery_mode else 'Inactive (Mains Power)'}")
            print(f"Battery Present                    : {'Detected' if battery_present else 'Not Detected'}")
            print(f"Temperature Sensor                 : {'Connected' if temp_sensor_connected else 'Not Connected (Check Sensor)'}")
            logging.info(f"Battery Mode Status: {'Active' if battery_mode else 'Inactive (Maians Power)'}")

        battery_temp_raw = None
        battery_capacity = None
        for address, (label, word_count, unit, scale) in registers.items():
            result = client.read_holding_registers(address, word_count, slave=slave_id)
            if result is None or isinstance(result, ModbusIOException) or result.isError():
                print(f"{label:<30}: ERROR")
                logging.error(f"Failed to read {label} at 0x{address:04X}")
                continue

            values = result.registers

            if label == "Device Name":
                if all(word == 0xFFFF for word in values):
                    print(f"{label:<30}: Unavailable")
                else:
                    name = ''.join(chr((word >> 8) & 0xFF) + chr(word & 0xFF) for word in values).strip('\x00')
                    print(f"{label:<30}: {name}")
            elif word_count == 1:
                raw = values[0]
                if raw == 0xFFFF:
                    print(f"{label:<30}: Unavailable (Raw: 0x{raw:04X})")
                else:
                    value = raw / scale if scale else raw
                    if unit == "K":
                        celsius = value - 273.15
                        print(f"{label:<30}: {celsius:.3f} °C (Raw: 0x{raw:04X})")
                        if label == "Battery Temperature":
                            battery_temp_raw = raw
                    else:
                        print(f"{label:<30}: {value:.2f} {unit or ''} (Raw: 0x{raw:04X})")
                        if label == "Battery Capacity":
                            battery_capacity = value
            else:
                raw_value = (values[0] << 16) + values[1]
                value = raw_value / scale if scale else raw_value
                print(f"{label:<30}: {value:.2f} {unit or ''} (Raw: 0x{raw_value:08X})")

        result = client.read_holding_registers(0x200A, 1, slave=slave_id)
        if result and not result.isError():
            battery_voltage = result.registers[0] / 1000
            soc = 100 * (battery_voltage - 20.4) / (27.5 - 20.4)
            soc = max(0, min(100, soc))
            print(f"Estimated Battery SOC               : {soc:.2f}% (Voltage: {battery_voltage:.2f} V, Capacity: {battery_capacity:.2f} 100mAh)")
            current_time = time.time()
            SOC_HISTORY.append((current_time, soc))
    
            # Display ASCII trend graph for last 10 minutes
            if SOC_HISTORY:
                print("\nSOC Trend (Last 10 Minutes):")
                time_window = current_time - 600
                relevant_data = [(t, s) for t, s in SOC_HISTORY if t >= time_window]
                soc_values = [s for _, s in relevant_data]
                if soc_values:
                    min_soc = min(soc_values)
                    max_soc = max(soc_values)
                    if max_soc - min_soc < 5:
                        mid = (max_soc + min_soc) / 2
                        min_soc = mid - 2.5
                        max_soc = mid + 2.5

                    cols = 50
                    rows = 10
                    chunk_size = max(1, len(soc_values) // cols)
                    avg_socs = [sum(soc_values[i:i+chunk_size]) / chunk_size for i in range(0, len(soc_values), chunk_size)][:cols]

                    for r in range(rows, -1, -1):
                        threshold = min_soc + (max_soc - min_soc) * (r / rows)
                        line = f"{threshold:5.1f}% | "
                        for soc in avg_socs:
                            line += '█' if soc >= threshold else ' '
                        print(line)
                    print("      +----------------------------------")
                    print("        0  1  2  3  4  5  6  7  8  9  10 min")

        client.close()

    except Exception as e:
        print(f"Unexpected error: {e}")
        logging.exception("Unexpected error occurred")

    time.sleep(5 if battery_mode else 10)
