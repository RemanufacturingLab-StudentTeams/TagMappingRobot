# runner.py
import json
import queue
import time
import sys
import traceback
from rfid_reader import RFIDReader
from processor import Processor
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

def main():
    cfg = load_config()

    q = queue.Queue(maxsize=cfg['max_size'])  # blocks when full

    processor = Processor(q,
                          batch_count=cfg['batch_count'],
                          batch_interval=cfg['batch_interval'],
                          mqtt_config=cfg['mqtt'],)

    processor.start()

    reader = RFIDReader(rfid_port = cfg['rfid_port'], antenna_power = cfg['antenna_power'], antenna_number = cfg['antenna_number'])
    try:
        reader.setup_connection()
    except Exception as e:
        print("Failed to open serial:", e)
        processor.stop()
        sys.exit(1)

    db.init_db()
    print("Runner: started. Press Ctrl+C to stop.")

    try:
        while True:
            starting_time = time.time()
            # get antenna position
            if cfg['interactive'].get('manual_antenna_position', True):
                ant_pos = antenna_pos_interactive()
            else:
                # use default placeholder if not interactive (should later be replaced by AGV/ROS2)
                ant_pos = (0.0, 0.0, 0.0, 0.0)
            location_time = time.time()
            print (f"getting antenna location took {location_time-starting_time}seconds")
            print ("preforming antenna cycle")
            measurements = reader.perform_measurement(ant_pos)
            antenna_time = time.time()
            #print (f"getting antenna data took {antenna_time-location_time}seconds")
            # check for unknown tags and optionally prompt user and retry
            if cfg['interactive'].get('ask_tag_position_on_first_seen', True):
                unknowns = [m['_unknown_tag'] for m in measurements if '_unknown_tag' in m]
                if unknowns:
                    for tid in unknowns:
                        x,y = tag_pos_interactive(tid)
                        reader.add_tag_location(tid, x, y)

            # push measurements into queue (blocks if full)
            q.put({
            "cycle": measurements,
            "timestamp": time.time()
            }, block=True)

            time.sleep(cfg.get('measurement_interval', 1.0))

    except KeyboardInterrupt:
        print("Runner: stopped by user.")
    except Exception:
        print("Runner: unexpected error:")
        traceback.print_exc()
    finally:
        print("Runner: shutting down...")
        processor.stop()
        processor.join(timeout=0)
        reader.close()
        sys.exit(0)

if __name__ == "__main__":
    main()
