#!/usr/bin/env python3
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
    """Load configuration file"""
    with open(CONFIG_FILE, 'r') as f:
        return json.load(f)

def antenna_pos_interactive():
    """
    manual measurment location input
    Returns (x, y, z, rot Z)
    """
    try:
        x = float(input("Antenna X [m]: "))
        y = float(input("Antenna Y [m]: "))
        z = float(input("Antenna Z [m]: "))
        rot = float(input("Antenna Rot Z [deg]: "))
        return (x, y, z, rot)
    except Exception as e:
        print(f"Failed to get manual antenna values:{e}, sending 0")
        return (0,0,0,0)

def tag_pos_interactive(tag_id):
    """
    Manual tag location input
    Returns (x, y)
    """
    try:
        x = float(input(f"Tag {tag_id} X [m]: "))
        y = float(input(f"Tag {tag_id} Y [m]: "))
        return (x, y)
    except Exception as e:
        print (f"faild to get manual tag location:{e}, using x,y = 0")
        return(0,0)

def init(cfg):
    """
    Parameters
    ----------
    cfg : config file

    Returns
    -------
    (data queue, processor, reader, mecabot)
    """
    q = queue.Queue(maxsize=cfg['max_size'])  # blocks when full
    
    processor = Processor(q,
                          batch_count=cfg['batch_count'],
                          mqtt_config=cfg['mqtt'],
                          interactive=cfg['interactive'],
                          lokalisation_config=cfg['lokalisation_config'])

    processor.start()
    
    reader = RFIDReader(rfid_port = cfg['rfid_port'], antenna_power = cfg['antenna_power'], antenna_number = cfg['antenna_number'])
    
    mecabot = None
    if (cfg['mecabot'].get('enabled')):
        mecabot = MQTTPoseReceiver(broker=cfg['mecabot'].get('broker'), topic=cfg['mecabot'].get('topic'), name=cfg['mecabot'].get('name'))
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
        
    db.init_db(File_name = cfg['lokalisation_config'].get['DATABASE_FILE'])
    print("Runner: started. Press Ctrl+C to stop.")
    return(q, processor, reader, mecabot)

def main():
    
    cfg = load_config() 	# get config file
    z = cfg['Z_value']  # set z value to preset z, for later use when implementing z axis in localisation
    q, processor, reader, mecabot = init(cfg)   # set all classes
    measurements = None     # set measuremnts to None to prevent undifend states
    
    try:
        while True:

            if cfg['mecabot'].get('enabled'):   #Get antenna position from mecabot when enabled if diabled default to manual
                ant_pos = mecabot.get_pose()    
                if ant_pos is not None:
                    x, y, yaw = ant_pos
                    ant_pos = x, y, z, yaw + 90     #add 90 degrees to rotation to compensate for rotated antenna mounting 
                print (f"measuring with pos:{ant_pos}")
            else:
                ant_pos = antenna_pos_interactive()
            if (ant_pos):   
                measurements = reader.perform_measurement(ant_pos)

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
