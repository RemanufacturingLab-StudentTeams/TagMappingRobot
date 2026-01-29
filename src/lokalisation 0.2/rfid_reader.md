# rfid_reader.py
### handles the communication with the rfid reader and antenna

### Features
- setup antenna connection
- get and extract data

### Functions
def __init__(self, rfid_port="/dev/ttyUSB0", antenna_power=3000, antenna_number=3):
- sets varibles

def calculate_checksum(self, command_bytes: bytes) -> bytes:
- calculates checksum used to validate data 

def send_command(self, command: bytes, timeout=0.2) -> str:
- send a read command to the rfid reader 

setup_connection(self):
- start connection with the rfid reader

def close(self):
- safely closes the connection with the rfid reader

def calculate_distance(x, y, z):
- calculates the distance to 0,0,0

def _extract_tag_details(self, lines):
- decodes recived data and seprates it to id, rssi and antenna

def perform_measurement(self, antenna_pos):
- main function to get data, calss alla functions to recive data and returns formated data