import os
import time
import traceback
import sys
from watchdog.observers import Observer
from watchdog_handler import TagFileHandler
#import paho.mqtt.client as mqtt

import tag_database as db
import check_csv as check
import rfid_reader as rfid

# --- Configuration ---
WATCH_FOLDER        = r'.\data'  
WRITE_FOLDER        = r'C:\Users\yoeri\De Haagse Hogeschool\Tag mapping robot_groups - Data'
PROCESSED_FOLDER    = os.path.join(WATCH_FOLDER, 'processed')
ERROR_FOLDER        = os.path.join(WATCH_FOLDER, 'faulty data')
FILE_PATTERN        = 'rfid_data_*.xlsx'
OUTPUT_FILE         = 'data1.csv'

# --- Values ---
interval = 5
treshold = 5
port = "COM3"



def main():
    
    db.init_db()
    
    
    check.init_csv(WRITE_FOLDER, OUTPUT_FILE)
    os.makedirs(PROCESSED_FOLDER, exist_ok=True)
    os.makedirs(ERROR_FOLDER, exist_ok=True)
    
    handler = TagFileHandler(WRITE_FOLDER, OUTPUT_FILE, PROCESSED_FOLDER, ERROR_FOLDER)
    observer = Observer()
    observer.schedule(handler, WATCH_FOLDER, recursive=False)
    observer.start()
    
    #rfid.setup_connection(port="COM3")
    #rfid.set_save_directory(WATCH_FOLDER) 

    print(f"Watching folder '{WATCH_FOLDER}' for new files...")

    try:
        while True:
            for i in range (treshold):
                print ("preforming measurment")
                time.sleep(interval)
                #rfid.perform_measurement()
            #rfid.save_to_excel()
    except KeyboardInterrupt:
        print("\nStopped by user.")

    except Exception as e:
        print(f"\nUnexpected error: {e}")
        traceback.print_exc()

    finally:
        observer.stop()
        observer.join()
        check.close()
        #rfid.close()
        
        sys.exit(0)

if __name__ == "__main__":
    main()