# rfid_reader.py
import time
import serial
import serial.tools.list_ports
from math import sqrt
from datetime import datetime
import os
import pandas as pd

TSL_BAUDRATE = 921600
TSL_BYTESIZE = 8
TSL_PARITY = 'N'
TSL_STOPBITS = 1
TSL_TIMEOUT = 0.1

save_directory = None

class RFIDReader:
    def __init__(self, rfid_port="/dev/ttyUSB0", antenna_power=3000, antenna_number=3):
        self.rfid_port = rfid_port
        self.antenna_power = antenna_power
        self.antenna_number = antenna_number
        self.ser = None
        self.tag_locations = {}  # tag_id -> (x,y)


    def calculate_checksum(self, command_bytes: bytes) -> bytes:
        c0, c1 = 0, 0
        for b in command_bytes:
            c0 = (c0 + b) % 255
            c1 = (c1 + c0) % 255
        checksum = (c1 << 8) | c0
        return checksum.to_bytes(2, byteorder='big')

    def send_command(self, command: bytes, timeout=0.2) -> str:
        """
        Send a command to the TSL module and read until EC: appears.
        Option A: Safe + Fast.
        """
        if not self.ser or not getattr(self.ser, "is_open", False):
            raise RuntimeError("Serial port not open")
    
        cmd = command + self.calculate_checksum(command) + b'\x0A'
    
        # Flush to ensure clean response
        self.ser.reset_input_buffer()
        self.ser.write(cmd)
    
        buffer = b""
        start = time.time()
    
        while True:
            # Read everything available
            n = self.ser.in_waiting
            if n > 0:
                buffer += self.ser.read(n)
    
                # Completed command response
                if b"EC:" in buffer:
                    break
    
            # Timeout
            if time.time() - start > timeout:
                break
    
            # prevent busy wait
            time.sleep(0.001)
    
        return buffer.decode("utf-8", errors="ignore")

    def setup_connection(self):
        self.ser = serial.Serial(
            self.rfid_port,
            baudrate=TSL_BAUDRATE,
            bytesize=TSL_BYTESIZE,
            parity=TSL_PARITY,
            stopbits=TSL_STOPBITS,
            timeout=0.05,
            write_timeout=0.05
        )
    
        # FAST antenna init (no debug, no metadata)
        inventory_command = f'$ir -bnx0 -sex0 -tax0 -slx0 -dtx1 -anx{self.antenna_number} -dbx{hex(self.antenna_power)[2:]} -trxFFFF'

        lines = self.send_command(inventory_command.encode()).split("\n")
    
        for line in lines:
            if line.startswith("EC:"):
                code = line.split(": ")[1]
                if code != "0":
                    raise Exception(f"Antenna setting failed. EC={code}")
    
        print(f"RFIDReader opened on {self.rfid_port}, antenna={self.antenna_number}")

    def close(self):
        try:
            if self.ser and getattr(self.ser, "is_open", False):
                self.ser.close()
        except Exception as e:
            print("RFIDReader.close error:", e)
        print("RFIDReader: closed")

    @staticmethod
    def calculate_distance(x, y, z):
        return round(sqrt(x**2 + y**2 + z**2), 3)
    
    def save_to_excel(self, custom_dir=None, data=None):
        timestamp = datetime.now().strftime('%d%m%y_%H%M%S')
        filename = f'rfid_data_{timestamp}.xlsx'
        
        directory = custom_dir or save_directory or os.getcwd()
        os.makedirs(directory, exist_ok=True)
        filepath = os.path.join(directory, filename)

        df = pd.DataFrame(data)

        with pd.ExcelWriter(filepath, engine='openpyxl') as writer:
            df.to_excel(writer, sheet_name='All Data', index=False)
            for tag_id, group_data in df.groupby('Tag ID'):
                sheet_name = str(tag_id)[:31]
                group_data.to_excel(writer, sheet_name=sheet_name, index=False)

        print(f'Raw data saved to {filename}')
        
    def set_save_directory(self, path)-> str:
        """Set a custom directory for saving output files."""
        global save_directory
        save_directory = path
        print(f"Save directory set to: {os.path.abspath(save_directory)}")

    def _extract_tag_details(self, lines):
        tags = []
        current_antenna = 'Unknown'
        for line in lines:
            line = line.strip()
            if not line:
                continue
            if line.startswith('BH:'):
                parts = line.split(': ', 1)[1].split(',')
                bank_header = {p.split('=')[0].strip(): p.split('=')[1].strip() for p in parts if '=' in p}
                current_antenna = bank_header.get('A', current_antenna)
            elif line.startswith('TR:'):
                parts = line.split(' | ')
                info = {p.split(': ', 1)[0].strip(): p.split(': ', 1)[1].strip() for p in parts if ': ' in p}
                try:
                    rssi = float(info.get('RI', '0'))/100.0
                except Exception:
                    rssi = 0.0
                tags.append({'Tag ID': info.get('EP', 'Unknown'), 'RSSI': rssi, 'Antenna': current_antenna})
            elif line.startswith('EC:'):
                code = line.split(': ', 1)[1]
                if code == str(10):
                    raise Exception("Antenna disconnected or impedance mismatch")
                elif code != str(0):
                    raise Exception(f"EC: {code}")
        return tags

    def perform_measurement(self, antenna_pos):
        """
        antenna_pos: tuple (ant_x, ant_y, ant_z, ant_rot_z), provided by caller.
        Returns list of measurement dicts.
        """
        if not self.ser:
            raise RuntimeError("Serial not initialized")
        ant_x, ant_y, ant_z, ant_rot_z = antenna_pos
        distance = self.calculate_distance(ant_x, ant_y, ant_z)
        try:
            start = time.time()
            raw = self.send_command(('$ba -go'.encode()))
            #print("RFID response time:", time.time() - start)

            lines = raw.split('\n')
            tags = self._extract_tag_details(lines)

            results = []
            for tag in tags:
                #print (f"detected ID:{tag}")
                results.append({
                    'Tag ID': tag['Tag ID'],
                    'RSSI': tag['RSSI'],
                    'Antenna': self.antenna_number,
                    'Antenna X [m]': ant_x,
                    'Antenna Y [m]': ant_y,
                    'Antenna Z [m]': ant_z,
                    'Antenna Rot Z [deg]': ant_rot_z,
                    'Distance [m]': distance,
                })
            return results
        except Exception as e:
            print("RFIDReader.perform_measurement error:", e)
            return []
