import os
import time
from watchdog.observers import Observer
from watchdog_handler import TagFileHandler

import tag_database as db
import check_csv as check
import rfid_reader

# --- Configuration ---
WATCH_FOLDER        = r'.\data'  
WRITE_FOLDER        = r'C:\Users\yoeri\De Haagse Hogeschool\Tag mapping robot_groups - Data'
PROCESSED_FOLDER    = os.path.join(WATCH_FOLDER, 'processed')
ERROR_FOLDER        = os.path.join(WATCH_FOLDER, 'faulty data')
FILE_PATTERN        = 'rfid_data_*.xlsx'
OUTPUT_FILE         = 'data1.csv'

db.init_db()
check.init_csv(WRITE_FOLDER, OUTPUT_FILE)
os.makedirs(PROCESSED_FOLDER, exist_ok=True)
os.makedirs(ERROR_FOLDER, exist_ok=True)


def main():
    
    handler = TagFileHandler(WRITE_FOLDER, OUTPUT_FILE, PROCESSED_FOLDER, ERROR_FOLDER)
    observer = Observer()
    observer.schedule(handler, WATCH_FOLDER, recursive=False)
    observer.start()
    
    rfid_reader.setup_connection(port="COM3")
    rfid_reader.set_save_directory(WATCH_FOLDER)
    rfid_reader.run_loop(interval=5, treshold=5)  # runs every 5 seconds
    
   

    print(f"Watching folder '{WATCH_FOLDER}' for new files...")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        observer.stop()
        print("\nStopped by user.")
    observer.join()

if __name__ == "__main__":
    main()