import os
import glob
import time
import pandas as pd
import shutil

import tag_database as db
import tag_localistation as taggLoc
import check_csv as check

# --- Configuration ---
WATCH_FOLDER = r'.\data'  
WRITE_FOLDER = r'C:\Users\yoeri\De Haagse Hogeschool\Tag mapping robot_groups - Data'
PROCESSED_FOLDER = os.path.join(WATCH_FOLDER, 'processed')
ERROR_FOLDER = os.path.join(WATCH_FOLDER, 'faulty data')
FILE_PATTERN = 'rfid_data_*.xlsx'
POLL_INTERVAL = 5  # seconds
OUTPUT_FILE = "data1.csv"

db.init_db()
os.makedirs(PROCESSED_FOLDER, exist_ok=True)
os.makedirs(ERROR_FOLDER, exist_ok=True)


def ensure_output_file():
    """Create the main CSV file if it does not exist."""
    file_path = os.path.join(WRITE_FOLDER, OUTPUT_FILE)
    if not os.path.exists(file_path):
        pd.DataFrame(columns=["ID", "X", "Y", "r", "w", "h"]).to_csv(file_path, index=False)
        print(f"Created new file: {OUTPUT_FILE}\nat: {file_path}")
    return file_path

def process_xlsx_file(xlsx_path, output_path):
    """Process a single Excel file and move it when done."""
    start = time.perf_counter()

    if not check.is_file_ready(xlsx_path):
        print(f"{xlsx_path} is not ready yet, will retry later.")
        return

    print(f"\nFound {xlsx_path}, processing...\n{'-' * 40}")

    try:
        with open(xlsx_path, 'rb') as f:
            tag_data = taggLoc.process_tag_data(f)  # adjust your function to accept a file-like object
        check.update_tag_data(tag_data, output_path)
    except Exception as e:
    # file is now closed, safe to move
        print(f"Error processing {xlsx_path}: {e}")
        try:
            dest_path = os.path.join(ERROR_FOLDER, os.path.basename(xlsx_path))
            shutil.move(xlsx_path, dest_path)
            print(f"Moved to {dest_path}")
        except Exception as e:
            print(f"Error moving {xlsx_path}: {e}")
        return

    # Move processed file
    try:
        dest_path = os.path.join(PROCESSED_FOLDER, os.path.basename(xlsx_path))
        shutil.move(xlsx_path, dest_path)
        print(f"Moved to {dest_path}")
    except Exception as e:
        print(f"Error moving {xlsx_path}: {e}")

    cycle_time = round(time.perf_counter() - start, 4)
    print(f"{'-' * 40}\nCycle time: {cycle_time} seconds")

def main():
    """Main loop to monitor and process files."""
    output_path = ensure_output_file()
    print(f"\nWatching folder '{WATCH_FOLDER}' for files matching '{FILE_PATTERN}'...\n")

    try:
        while True:
            xlsx_files = glob.glob(os.path.join(WATCH_FOLDER, FILE_PATTERN))

            if not xlsx_files:
                print("No new files found.")
            else:
                # Sort by modification time (oldest first)
                xlsx_files.sort(key=lambda f: os.path.getmtime(f))

                for xlsx_path in xlsx_files:
                    process_xlsx_file(xlsx_path, output_path)

            time.sleep(POLL_INTERVAL)

    except KeyboardInterrupt:
        print("\nProcessing stopped by user.")
    except Exception as e:
        print(f"\nUnexpected error: {e}")
        
if __name__ == "__main__":
    main()