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
 Last Updated : 2025-07-31
 Version      : 1.2
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
    - Displays user battery mode time in minutes (feature barely works, due to the fact that the UPS does not support this register and it is not implemented in the device)
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
# Imports, we can remove the pymodbus import if we want to keep it simple, but it is used for Modbus communication
# the versions of the libraries used are:
# pymodbus==3.0.0   
# configparser==5.3.0
# logging is used for logging errors and connection status'

import time
import os
import csv
import configparser
import logging
from pymodbus.client import ModbusTcpClient
from pymodbus.exceptions import ModbusIOException
from collections import deque

VALUES_LOG_FILE = "ups_values.csv"
LOG_INTERVAL = 5  # seconds
last_log_time = 0  # keeps track of the last log time

# This function ensures that the CSV file has a header row with the correct column names.
# If the file does not exist, it creates it with the header. If it exists but the header is missing or incorrect, it rewrites the header and keeps the existing data.'

def ensure_csv_header(filename):
    header = [
        'Timestamp',
        'Battery Voltage (V)',
        'Output Current (mA)',
        'Battery Temp (°C)',
        'Device Temp (°C)',
        'Battery Current (mA)',
        'SOC (%)'
    ]
    # Here we check if the file exists, if not we create it with the header
    # If it exists but the header is missing or incorrect, we rewrite the header and keep
    # the existing data
    # This is to ensure that the CSV file has the correct header and does not get overwritten   
    # Loop to ensure the header is present in the CSV file
    # This is useful if the script is run multiple times and we want to ensure that the, header is always present in the CSV file
    # We can remove this if we want to keep it simple, but it is useful to ensure that the CSV file has the correct header and does not get overwritten  

    if not os.path.isfile(filename):
        with open(filename, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(header)
    else:
        with open(filename, mode='r', newline='') as f:
            first_line = f.readline()
            if first_line.strip() != ','.join(header):
                lines = f.readlines()
                with open(filename, mode='w', newline='') as fw:
                    writer = csv.writer(fw)
                    writer.writerow(header)
                    fw.writelines(lines)

# What this does is it checks if the CSV file exists, if not it creates it with the header
# If it exists but the header is missing or incorrect, it rewrites the header and keeps the existing data
ensure_csv_header(VALUES_LOG_FILE)#

# Configure logging
logging.basicConfig(
    filename='modbus_errors.log',
    level=logging.INFO,
    format='%(asctime)s %(levelname)s: %(message)s'
)

# Load Modbus TCP settings, these are read from a config file (config.ini)
# The config file should contain a section [MODBUS] with options ip_address, port, and slave_id (this is default 192)

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

# Define registers, these are the registers that we will read from the UPS
# These are from the Phoenix Contact TRIO UPS documentation, no SOC register is available, so we estimate it based on battery voltage
# The register addresses are in hexadecimal format, and the values are tuples containing:
# - Label: Name of the register
# - Word Count: Number of words to read (1 or 2)
# - Unit: Unit of the value (e.g., V, mA, K)
# The - Scale: Scale factor for the value (e.g., 1000 for voltage in mV, 1 for current in mA)
# The values are read as raw values and then converted to the appropriate unit using the scale factor, because the UPS uses a different scale for each register, No idea why they did this, but it is what it is ;D 
# The registers are read as holding registers, which are 16-bit values, so we read them in pairs if the word count is 2

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

# Initialize deque for SOC history, this will store the last 10 minutes of SOC data 
SOC_HISTORY = deque(maxlen=600)
# Ensure the deque is empty at start and does not contain any old data, connecting to the UPS might take a while 
# This part of the code does the folllowing:
# - It initializes a deque with a maximum length of 600, which will store the last 10 minutes of SOC data (1 entry per second), found it in the documentation of the Phoenix Contact TRIO UPS
# - The deque is used to store the SOC data in a circular buffer, so that we can easily access the last 10 minutes of data
# - The deque is initialized as empty, so that it does not contain any old data when the script starts, it gives trash data if we do not clear it
# - The maxlen parameter ensures that the deque does not grow beyond 600 entries, which is useful to keep the memory usage low and to ensure that we only keep the last 10 minutes of data
# Initialize last_log_time to the current time, so that we can log the first values immediately
SOC_HISTORY.clear()  # Clear any old data
last_log_time = time.time()
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
            logging.info(f"Battery Mode Status: {'Active' if battery_mode else 'Inactive (Mains Power)'}")

        battery_temp_raw = None # Initialize to None because it might not be read 
        battery_capacity = None
        for address, (label, word_count, unit, scale) in registers.items(): 
            result = client.read_holding_registers(address, word_count, slave=slave_id)
            if result is None or isinstance(result, ModbusIOException) or result.isError():
                print(f"{label:<30}: ERROR")
                logging.error(f"Failed to read {label} at 0x{address:04X}")
                continue
            # If result is valid, we can read the registers
            values = result.registers
            # Here we check if the values are empty, if so we print Unavailable and then we continue to the next register, we also print raw value if available 
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
        # Here we read the battery voltage register, which is used to estimate the SOC 
        if result and not result.isError():
            battery_voltage = result.registers[0] / 1000
            soc = 100 * (battery_voltage - 20.4) / (27.5 - 20.4)
            soc = max(0, min(100, soc))
            print(f"Estimated Battery SOC               : {soc:.2f}% (Voltage: {battery_voltage:.2f} V, Capacity: {battery_capacity:.2f} 100mAh)")
            current_time = time.time()
            SOC_HISTORY.append((current_time, soc))
    
            # Display ASCII trend graph for last 10 minutes, could be improved with more advanced graphing libraries but we can remove if we want to keep it simple
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
    # We put it on none beause it might not be read in the first loop, so we can read it in the second loop, actually we can remove this if we want to keep it simple
        device_temp_c = None
        battery_temp_c = None
        battery_current = None
        output_current = None

        # Here we read the remaining registers that are not in the first loop 
        for address, (label, word_count, unit, scale) in registers.items():
            if label == "Battery Temperature" and battery_temp_raw:
                battery_temp_c = battery_temp_raw / scale - 273.15
            if label == "Device Temperature":
                result = client.read_holding_registers(address, word_count, slave=slave_id)
                if result and not result.isError():
                    raw = result.registers[0]
                    device_temp_c = raw / scale - 273.15
            if label == "Battery Current":
                result = client.read_holding_registers(address, word_count, slave=slave_id)
                if result and not result.isError():
                    battery_current = result.registers[0] / scale
            if label == "Output Current":
                result = client.read_holding_registers(address, word_count, slave=slave_id)
                if result and not result.isError():
                    output_current = result.registers[0] / scale


        if time.time() - last_log_time > LOG_INTERVAL:
            print("Logging to CSV")  # For debugging, see if logging triggers

        # Log every x seconds
        if time.time() - last_log_time > LOG_INTERVAL:
            last_log_time = time.time()
            with open(VALUES_LOG_FILE, mode='a', newline='') as file:
                writer = csv.writer(file)
                # Here we write the values to the CSV file and check if they are None, if so we write an empty string and not a number
                ensure_csv_header(VALUES_LOG_FILE)  # Ensure header is present this will not write the header again if it is already present 
                writer.writerow([
                    time.strftime('%Y-%m-%d %H:%M:%S'),
                    round(battery_voltage, 3) if battery_voltage is not None else '',
                    round(battery_temp_c, 2) if battery_temp_c is not None else '',
                    round(device_temp_c, 2) if device_temp_c is not None else '',
                    round(battery_current, 2) if battery_current is not None else '',
                    round(output_current, 2) if output_current is not None else '',
                    round(soc, 2) if soc is not None else '',
                ])


        client.close()

    except Exception as e:
        print(f"Unexpected error: {e}")
        logging.exception("Unexpected error occurred")
        time.sleep(5 if battery_mode else 10)# Sleep longer if not in battery mode

# Check connection health and retry if necessary if not connected then we will retry the connection 
    if not client.is_socket_open():
        print(f"[{time.strftime('%H:%M:%S')}] Lost connection. Retrying...")
        logging.warning("Modbus connection lost, retrying...")
        time.sleep(3)
    else:
        print(f"[{time.strftime('%H:%M:%S')}] Connection healthy.")
        logging.info("Modbus connection healthy.")
