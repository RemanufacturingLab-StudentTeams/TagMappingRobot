import os

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

DATA_DIR = os.path.join(BASE_DIR, "data")
ASSETS_DIR = os.path.join(BASE_DIR, "assets")

os.makedirs(DATA_DIR, exist_ok=True)

NAME_FILE = os.path.join(DATA_DIR, "tag_names.csv")
DB_PATH = os.path.join(DATA_DIR, "tag_data_HMI.db")

MQTT_BROKER = "localhost"
MQTT_PORT = 1883
MQTT_KEEPALIVE = 60
MQTT_TOPIC = "HMI/topic"

UPDATE_INTERVAL_MS = 2000

LAYOUT_IMAGE = {'source': '/assets/lokaal.png', 'xref': 'x', 'yref': 'y', 'x': -5.21, 'y': 5.48, 'sizex': 10.700000000000001, 'sizey': 9.25, 'sizing': 'stretch', 'layer': 'below', 'opacity': 0.8}

AXIS_RANGE_CONFIG = {
    "x" : [-5.21, 5.490000000000001], 
    "y" : [-3.77, 5.48]
    
    
}
