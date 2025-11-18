import serial
import serial.tools.list_ports
import signal
import sys
import time
import pandas as pd
import os
import simulation_algoritme as sim
from datetime import datetime
from math import sqrt
from watchdog.observers import Observer


# Serial port configuration
tsl_name = 'TSL RAIN RFID MODULE'
tsl_baudrate = 921600
tsl_bytesize = 8
tsl_parity = 'N'
tsl_stopbits = 1
tsl_timeout = 2

# Default antenna settings
antenna_port = 3
antenna_power = 300

ser = None
df = None
tag_locations = {}
save_directory = None  # Default = current working directory
observer = Observer()


# ----------------------- Utility Functions -----------------------

def find_port(portname):
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if portname in str(port.description):
            print("Port found:", port.device)
            return port.device
    raise Exception("Port not found. Check the connection.")


def calculate_checksum(command):
    c0, c1 = 0, 0
    for char in command:
        c0 = (c0 + char) % 255
        c1 = (c1 + c0) % 255
    checksum = (c1 << 8) | c0
    return checksum.to_bytes(2, byteorder='big')


def send_command(command):
    command_with_checksum = command + calculate_checksum(command)
    ser.write(command_with_checksum + b'\x0A')
    response = ser.read(3000)
    return response.decode('utf-8')


def init_antenna(antenna_number, power):
    inventory_command = f'$ir -bnx0 -sex0 -tax0 -slx0 -dtx1 -anx{antenna_number} -dbx{hex(power)[2:]} -trxFFFF'
    print("Initializing antenna:", inventory_command)
    lines = send_command(inventory_command.encode()).split('\n')
    time.sleep(1)
    for line in lines:
        if line.startswith('EC:'):
            error_code = line.split(': ', 1)[1]
            if error_code != '0':
                raise Exception(f"Antenna setting failed. EC: {error_code}")
            print(f"Antenna set to {antenna_number}")


def extract_tag_details(lines):
    tags = []
    current_antenna = 'Unknown'
    for line in lines:
        if line.startswith('BH:'):
            bh_details = line.split(': ', 1)[1].split(',')
            bank_header = {part.split('=')[0].strip(): part.split('=')[1].strip() for part in bh_details}
            current_antenna = bank_header.get('A', 'Unknown')
        elif line.startswith('TR:'):
            tag_details = line.split(' | ')
            tag_info = {d.split(': ', 1)[0].strip(): d.split(': ', 1)[1].strip() for d in tag_details if ': ' in d}
            rssi = float(tag_info.get('RI', '0')) / 100
            tags.append({
                'Tag ID': tag_info.get('EP', 'Unknown'),
                'RSSI': rssi,
                'Antenna': current_antenna
            })
        elif line.startswith('EC:'):
            error_code = line.split(': ', 1)[1]
            if error_code == str(10):
                raise Exception("Antenna disconnected or impedance mismatch")
            elif error_code != str(0):
                raise Exception(f"EC: {error_code}")
    return tags

def set_save_directory(path):
    """Set a custom directory for saving output files."""
    global save_directory
    save_directory = path
    print(f"Save directory set to: {os.path.abspath(save_directory)}")


def signal_handler(sig, frame):
    #save_to_excel()
    #ser.close()
    sys.exit(0)
    observer.stop()


def save_to_excel(custom_dir=None):
    global df
    timestamp = datetime.now().strftime('%d%m%y_%H%M%S')
    filename = f'rfid_data_{timestamp}.xlsx'
    
    directory = custom_dir or save_directory or os.getcwd()
    os.makedirs(directory, exist_ok=True)
    filepath = os.path.join(directory, filename)

    with pd.ExcelWriter(filepath, engine='openpyxl') as writer:
        df.to_excel(writer, sheet_name='All Data', index=False)
        for tag_id, group_data in df.groupby('Tag ID'):
            sheet_name = str(tag_id)[:31]
            group_data.to_excel(writer, sheet_name=sheet_name, index=False)
            
    df = df.iloc[0:0]
    print(f'Results saved to {filename}')


def calculate_distance(x, y, z):
    return round(sqrt(x**2 + y**2 + z**2), 3)


def get_valid_float(prompt):
    while True:
        try:
            return float(input(prompt))
        except ValueError:
            print("Error: Please enter a valid number.")


# ----------------------- Core Logic -----------------------

def setup_connection(port='COM3'):
    """Initialize the serial connection and antenna."""
    global ser, df, tag_locations
    """
    try:
        ser = serial.Serial(port, baudrate=tsl_baudrate, bytesize=tsl_bytesize,
                            parity=tsl_parity, stopbits=tsl_stopbits, timeout=tsl_timeout)
    except Exception as e:
        raise Exception(f"Error opening serial port: {e}")
        """
    antenna = antenna_port
    power = antenna_power
    #init_antenna(antenna, power)

    df = pd.DataFrame(columns=[
        'Tag ID', 'RSSI', 'Antenna',
        'Antenna X [m]', 'Antenna Y [m]', 'Antenna Z [m]',
        'Antenna Rot Z [deg]', 'Distance [m]',
        'Tag X [m]', 'Tag Y [m]'
    ])

    tag_locations = {}
    signal.signal(signal.SIGINT, signal_handler)
    print("Setup complete. RFID reader ready.")


def perform_measurement(measurement_count, i):
    """Perform a single RFID measurement cycle."""
    global df, tag_locations
    
    values = sim.get_values('mecabot',i)
    antenna_x = values ['x']
    antenna_y = values['y']
    antenna_z = values['z']
    antenna_rot_z = values['rot']
    distance = calculate_distance(antenna_x, antenna_y, antenna_z)

    try:
        # tag_data = send_command('$ba -go'.encode())
        
        # tag_lines = tag_data.split('\n')
        # tags = extract_tag_details(tag_lines)
        tag = sim.get_values('antenna', i)

        #for tag in tags:
        tag_id = tag['Tag ID']

        tag_data = {
            'Tag ID': tag_id,
            'RSSI': tag['RSSI'],
            'Antenna': 3,
            'Antenna X [m]': antenna_x,
            'Antenna Y [m]': antenna_y,
            'Antenna Z [m]': antenna_z,
            'Antenna Rot Z [deg]': antenna_rot_z,
            'Distance [m]': distance,
        }
        df = df._append(tag_data, ignore_index=True)
        measurement_count += 1
        print("\nCurrent measurements:")
        print(pd.DataFrame([tag_data]))
        #print(f"\nTotal measurements recorded: {len(df)}")
        #print(f"Current distance from origin: {distance:.2f} meters")

    except Exception as e:
        print(f"Error during measurement: {e}")
        print("Continuing to next measurement...")
    return (measurement_count)


def run_loop(interval, treshold):
    """Continuously perform measurements every `interval` seconds with error handling."""
    print(f"Starting measurement loop every {interval} seconds. Press Ctrl+C to stop.\n")
    measurement_count = 0
    i = 0
    try:
        while True:
            try:
                measurement_count = perform_measurement(measurement_count, i)
                i = 1 + i
                if (measurement_count >= treshold):
                    measurement_count = 0
                    save_to_excel()
                    time.sleep(6)
                    input("Press Enter to continue to the next measurement or Ctrl+C to stop.")
                if (i == sim.amount_data()):
                    save_to_excel()
                    #ser.close()
                    sys.exit(0)
            except Exception as e:
                print(f"\nError during measurement cycle: {e}")
                print("Skipping this cycle and continuing...\n")
            
            print(f"Waiting {interval} seconds before next measurement...\n")
            time.sleep(interval)

    except Exception as e:
        print(f"\nUnexpected fatal error: {e}")
        save_to_excel()
        observer.stop()
        observer.join()
        if ser and ser.is_open:
            ser.close()
        raise  # Re-raise to let external caller handle it if needed
