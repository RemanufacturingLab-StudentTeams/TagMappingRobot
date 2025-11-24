import os
import shutil
import threading
from watchdog.events import FileSystemEventHandler
import tag_localistation as taggLoc
import check_csv as check
import time
import paho.mqtt.client as mqtt


class TagFileHandler(FileSystemEventHandler):
    """Handles new or modified tag XLSX files."""
    def __init__(self, write_folder: str, output_folder: str, processed_folder: str, error_folder: str):
        super().__init__()
        self.broker = "10.35.4.50"
        #self.broker = "192.168.137.86"

        self.port = 1883

        self.topic = "test/topic"

        self.client = mqtt.Client()
        #self.client.connect(self.broker, self.port, 60)
        self.OUTPUT_FILE = os.path.join(write_folder, output_folder)
        self.PROCESSED_FOLDER = processed_folder
        self.ERROR_FOLDER = error_folder
        self._recent_files = {}  # track last processed time per file
        self.COOLDOWN = 3  # seconds before the same file can be reprocessed

    # ---------------------------------------------------------------------
    # Event hooks
    # ---------------------------------------------------------------------
    def on_created(self, event):
        """Triggered when a new file is created."""
        if not event.is_directory and event.src_path.endswith(".xlsx"):
            self._schedule_process(event.src_path)
            
    def _schedule_process(self, path: str):
        """
        Schedule a delayed file processing, with debounce protection.
        """
        filename = os.path.basename(path)
        now = time.time()
    
        # Skip if recently processed
        if filename in self._recent_files and (now - self._recent_files[filename]) < self.COOLDOWN:
            return
    
        self._recent_files[filename] = now
        threading.Thread(target=self._delayed_process, args=(path,), daemon=True).start()
    
    def _delayed_process(self, path: str):
        """Delay slightly before processing to ensure file is fully written."""
        time.sleep(0.5)
        for _ in range(5):  # Retry up to 5 times if file not ready
            if check.is_file_ready(path):
                self.process_file(path)
                return
            time.sleep(0.5)
        print(f"Skipped (still locked): {path}")

    def process_file(self, xlsx_path):
        # Wait until file is ready
        if not check.is_file_ready(xlsx_path):
            print(f"Skipping (not ready): {xlsx_path}")
            return

        print(f"Processing {xlsx_path}...\n{'-'*40}")
        start = time.perf_counter()

        try:
            with open(xlsx_path, 'rb') as f:
                tag_data = taggLoc.process_tag_data(f)
            check.update_tag_data(tag_data, self.OUTPUT_FILE, self.client)
        except Exception as e:
            print(f"Error processing {xlsx_path}: {e}")
            try:
                shutil.move(xlsx_path, os.path.join(self.ERROR_FOLDER, os.path.basename(xlsx_path)))
            except Exception as e2:
                print(f"Error moving faulty file: {e2}")
            return
        
        # Move successfully processed file
        cycle_time = round(time.perf_counter() - start, 4) 
        print(f"{'-' * 40}\nCycle time: {cycle_time} seconds \n")
        try:
            shutil.move(xlsx_path, os.path.join(self.PROCESSED_FOLDER, os.path.basename(xlsx_path)))
        except Exception as e:
            print(f"Error moving processed file: {e}")
