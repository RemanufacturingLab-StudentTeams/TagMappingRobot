import serial
import serial.tools.list_ports
import signal
import sys
import time
import pandas as pd
from datetime import datetime
from math import sqrt

# Serial port configuration for an RFID reader
tsl_name = 'TSL RAIN RFID MODULE'
tsl_baudrate = 921600
tsl_bytesize = 8
tsl_parity = 'N'
tsl_stopbits = 1
tsl_timeout = 2

def find_port(portname):
    """Scan for a COM port that includes the given port name and return its device name."""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if portname in str(port.description):
            print("Port found: " + port.device)
            return port.device
    raise Exception("Port not found. Check the connection.")

def calculate_checksum(command):
    """Calculate Fletcher-16 checksum for a command."""
    c0, c1 = 0, 0
    for char in command:
        c0 = (c0 + char) % 255
        c1 = (c1 + c0) % 255
    checksum = (c1 << 8) | c0
    return checksum.to_bytes(2, byteorder='big')

def send_command(command):
    """Send a command with an appended checksum to the RFID device and read the response."""
    command_with_checksum = command + calculate_checksum(command)
    ser.write(command_with_checksum + b'\x0A')  # End command with line feed
    response = ser.read(3000)  # Read the response (up to 3000 bytes)
    return response.decode('utf-8')

def init(antenna_number, power):
    """Set the antenna number on the RFID reader."""
    inventory_command = f'$ir -bnx0 -sex0 -tax0 -slx0 -dtx1 -anx{antenna_number} -dbx{hex(power)[2:]} -trxFFFF'
    print(inventory_command)
    lines = send_command(inventory_command.encode()).split('\n')
    time.sleep(1)  # Delay for command processing
    for line in lines:
        if line.startswith('EC:'):
            error_code = line.split(': ', 1)[1]
            if error_code != '0':
                raise Exception(f"Antenna setting to {antenna_number} failed. EC: {error_code}")
            else:
                print(f"Antenna set to {antenna_number}")

def extract_tag_details(lines):
    """Extract and return tag details from response lines."""
    tags = []
    current_antenna = 'Unknown'
    for line in lines:
        if line.startswith('BH:'):
            bh_details = line.split(': ', 1)[1].split(',')
            bank_header = {part.split('=')[0].strip(): part.split('=')[1].strip() for part in bh_details}
            current_antenna = bank_header.get('A', 'Unknown')
        elif line.startswith('TR:'):
            tag_details = line.split(' | ')
            tag_info = {detail.split(': ', 1)[0].strip(): detail.split(': ', 1)[1].strip() for detail in tag_details if ': ' in detail}
            # Convert RSSI to float and divide by 100
            rssi = float(tag_info.get('RI', '0')) / 100
            tags.append({
                'Tag ID': tag_info.get('EP', 'Unknown'),
                'RSSI': rssi,
                'Antenna': current_antenna
            })
        elif line.startswith('EC:'):
            error_code = line.split(': ', 1)[1]
            if error_code == str(10):
                raise Exception("Disconnected, poorly-connected or mismatched impedance of the antenna")
            elif error_code != str(0):
                raise Exception(f"EC: {error_code}")
    return tags

def signal_handler(sig, frame):
    #save data to excel when program ends
    timestamp = datetime.now().strftime('%d%m%y_%H%M%S')
    filename = f'rfid_data_{timestamp}.xlsx'
    print(f'\nSaving results to {filename}...')
    
    # Create Excel writer object
    with pd.ExcelWriter(filename, engine='openpyxl') as writer:
        # First save all data to a sheet named 'All Data'
        df.to_excel(writer, sheet_name='All Data', index=False)
        
        # Group data by Tag ID and save each group to a separate sheet
        for tag_id, group_data in df.groupby('Tag ID'):
            # Clean the tag_id to make it a valid Excel sheet name
            sheet_name = str(tag_id)[:31]  # Excel sheet names limited to 31 chars
            group_data.to_excel(writer, sheet_name=sheet_name, index=False)
    
    print(f'Results saved to {filename}')
    print('\nData has been organized by Tag ID in separate sheets.')
    ser.close()
    sys.exit(0)

def calculate_distance(x, y, z):
    """Calculate the Euclidean distance from origin (0,0,0) to point (x,y,z)."""
    return round(sqrt(x**2 + y**2 + z**2), 3)  # Round to 3 decimal places

def get_valid_antenna():
    """Get valid antenna input from user."""
    while True:
        try:
            antenna = int(input("Please enter the antenna port (1-4): "))
            if 1 <= antenna <= 4:
                return antenna
            print("Error: Antenna port must be between 1 and 4. Please try again.")
        except ValueError:
            print("Error: Please enter a valid number.")

def get_valid_power():
    """Get valid power input from user."""
    while True:
        try:
            power = int(input("Please enter the reader power (0-3000): "))
            if 0 <= power <= 3000:
                return power
            print("Error: Power must be between 0 and 3000. Please try again.")
        except ValueError:
            print("Error: Please enter a valid number.")

def get_valid_float(prompt):
    """Get valid float input from user."""
    while True:
        try:
            value = float(input(prompt))
            return value
        except ValueError:
            print("Error: Please enter a valid number.")

if __name__ == "__main__":
    try:
        ser = serial.Serial('COM3', baudrate=tsl_baudrate, bytesize=tsl_bytesize, 
                          parity=tsl_parity, stopbits=tsl_stopbits, timeout=tsl_timeout)
    except Exception as e:
        print(f"Error opening serial port: {e}")
        exit(1)

    # Initialize antenna and power settings
    antenna = get_valid_antenna()
    power = get_valid_power()
    
    while True:
        try:
            init(antenna, power)
            break
        except Exception as e:
            print(f"Error initializing antenna: {e}")
            antenna = get_valid_antenna()
            power = get_valid_power()

    # DataFrame to store the results
    df = pd.DataFrame(columns=['Tag ID', 'RSSI', 'Antenna', 
                             'Antenna X [m]', 'Antenna Y [m]', 'Antenna Z [m]', 
                             'Antenna Rot Z [deg]', 'Distance [m]',
                             'Tag X [m]', 'Tag Y [m]'])  # Added Tag X and Y columns

    signal.signal(signal.SIGINT, signal_handler) #when CTRL+C pressed, run signal_handler

    # Dictionary to store tag locations
    tag_locations = {}

    while True:
        # Get antenna position and rotation
        antenna_x = get_valid_float("Please enter the Antenna X position [m]: ")
        antenna_y = get_valid_float("Please enter the Antenna Y position [m]: ")
        antenna_z = get_valid_float("Please enter the Antenna Z position [m]: ")
        antenna_rot_z = get_valid_float("Please enter the Antenna Z rotation [deg]: ")

        # Calculate distance from origin
        distance = calculate_distance(antenna_x, antenna_y, antenna_z)

        try:
            # Perform inventory
            tag_data = send_command('$ba -go'.encode())
            tag_lines = tag_data.split('\n')
            tags = extract_tag_details(tag_lines)

            # Add measurements to DataFrame
            for tag in tags:
                tag_id = tag['Tag ID']
                
                # If this is a new tag, get its location
                if tag_id not in tag_locations:
                    print(f"\nNew Tag ID detected: {tag_id}")
                    tag_x = get_valid_float(f"Please enter the Tag X position [m] for Tag {tag_id}: ")
                    tag_y = get_valid_float(f"Please enter the Tag Y position [m] for Tag {tag_id}: ")
                    tag_locations[tag_id] = (tag_x, tag_y)
                
                # Get tag location from dictionary
                tag_x, tag_y = tag_locations[tag_id]
                
                tag_data = {
                    'Tag ID': tag_id,
                    'RSSI': tag['RSSI'],
                    'Antenna': tag['Antenna'],
                    'Antenna X [m]': antenna_x,
                    'Antenna Y [m]': antenna_y,
                    'Antenna Z [m]': antenna_z,
                    'Antenna Rot Z [deg]': antenna_rot_z,
                    'Distance [m]': distance,
                    'Tag X [m]': tag_x,
                    'Tag Y [m]': tag_y
                }
                df = df._append(tag_data, ignore_index=True)

            print("\nCurrent measurements:")
            print(pd.DataFrame([tag_data]))
            print(f"\nTotal measurements recorded: {len(df)}")
            print(f"Current distance from origin: {distance:.2f} meters")
        
        except Exception as e:
            print(f"Error during measurement: {e}")
            print("Continuing to next measurement...")
            continue

        # Ask user to proceed to the next measurement
        input("Press Enter to continue to the next measurement or Ctrl+C to stop.")
