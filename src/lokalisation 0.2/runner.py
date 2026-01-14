# runner.py
import json
import queue
import time
import sys
import numpy as np
import traceback
from rfid_reader import RFIDReader
from processor import Processor
from mqtt_listner import MQTTPoseReceiver
import db



CONFIG_FILE = "config.json"

def load_config():
    with open(CONFIG_FILE, 'r') as f:
        return json.load(f)

def antenna_pos_interactive():
    try:
        x = float(input("Antenna X [m]: "))
        y = float(input("Antenna Y [m]: "))
        z = float(input("Antenna Z [m]: "))
        rot = float(input("Antenna Rot Z [deg]: "))
        return (x, y, z, rot)
    except Exception as e:
        print(f"Failed to get antenna values:{e}, sending 0")
        return (0,0,0,0)

def tag_pos_interactive(tag_id):
    x = float(input(f"Tag {tag_id} X [m]: "))
    y = float(input(f"Tag {tag_id} Y [m]: "))
    return (x, y)

def init(cfg):
    q = queue.Queue(maxsize=cfg['max_size'])  # blocks when full
    
    processor = Processor(q,
                          batch_count=cfg['batch_count'],
                          mqtt_config=cfg['mqtt'],
                          interactive=cfg['interactive'],
                          lokalisation_config=cfg['lokalisation_config'])

    processor.start()
    
    reader = RFIDReader(rfid_port = cfg['rfid_port'], antenna_power = cfg['antenna_power'], antenna_number = cfg['antenna_number'])
    reader.set_save_directory(cfg['interactive'].get('raw_folder')) 
    
    mecabot = None
    if (cfg['mecabot'].get('enabled')):
        mecabot = MQTTPoseReceiver(broker=cfg['mecabot'].get('broker'), topic=cfg['mecabot'].get('topic'))
        try:
            mecabot.start()
            print ("mecabot started")
        except Exception as e:
            print("Failed to connect to mecabot:", e)
            processor.stop()
            sys.exit(1)
    try:
        reader.setup_connection()
    except Exception as e:
        print("Failed to open serial:", e)
        processor.stop()
        sys.exit(1)
        
    db.init_db()
    print("Runner: started. Press Ctrl+C to stop.")
    return(q, processor, reader, mecabot)

def get_delta(new, old):
    x = abs(new[0]-old[0])
    y = abs(new[1]-old[1])
    return (np.sqrt(x*x+y*y))

def main():
    
    cfg = load_config()
    z = cfg['Z_value']
    q, processor, reader, mecabot = init(cfg)
    measurements = None
    pose = None

    try:
        while True:

            if cfg['mecabot'].get('enabled'):
                pose = mecabot.get_pose()    
                
                x, y, yaw = pose
                ant_pos = x, y, z, yaw - 90
                print (f"measuring with pos:{pose}")
            else:
                ant_pos = antenna_pos_interactive()
            if (ant_pos):
                measurements = reader.perform_measurement(ant_pos)

            # check for unknown tags and optionally prompt user
            if cfg['interactive'].get('ask_tag_position_on_first_seen', True):
                unknowns = [m['_unknown_tag'] for m in measurements if '_unknown_tag' in m]
                if unknowns:
                    for ID in unknowns:
                        x,y = tag_pos_interactive(ID)
                        reader.add_tag_location(ID, x, y)

            # push measurements into queue (blocks if full)
            if (measurements):
                for m in measurements:
                    tag = m.get("Tag ID")
                    print ("detected tag:", tag)
                q.put({
                "cycle": measurements,
                "timestamp": time.time()
                }, block=True)

            time.sleep(cfg.get('measurement_interval', 1.0))

    except KeyboardInterrupt:
        print("\nRunner: stopped by user.")
    except Exception as e:
        print("\nRunner: unexpected error:", e)
        traceback.print_exc()
    finally:
        print("Runner: shutting down...")
        if (cfg['mecabot'].get('enabled')):
            mecabot.stop()
        processor.stop()
        processor.join(timeout=0)
        reader.close()

if __name__ == "__main__":
    main()
