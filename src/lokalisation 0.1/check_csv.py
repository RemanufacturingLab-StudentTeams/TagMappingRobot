import pandas as pd
import time
import os
import paho.mqtt.client as mqtt
import json 

broker = "10.35.4.50"
port = 1883
topic = "test/topic"
client = mqtt.Client()

def init_csv(WRITE_FOLDER, OUTPUT_FILE):
    """Create the main CSV file if it does not exist."""
    file_path = os.path.join(WRITE_FOLDER, OUTPUT_FILE)

    if not os.path.exists(file_path):
        pd.DataFrame(columns=["ID", "X", "Y", "r", "w", "h"]).to_csv(file_path, index=False)
        print(f"Created new file: {OUTPUT_FILE}\nat: {file_path}")    

def is_file_ready(file_path, retries=3, delay=0.2):
    """Check if a file is ready to be read (not being written)."""
    for _ in range(retries):
        try:
            pd.read_excel(file_path, nrows=1)
            return True
        except Exception:
            time.sleep(delay)
    return False

def update_tag_data(tag_data, file_path, client):
    print (tag_data)
    message = json.dumps(tag_data)
    print (message)
    client.publish(topic, message)
    current_tag_data = pd.read_csv(file_path)
    
    for tags in tag_data:  
        tag_ID = tags["ID"]
        
        if tag_ID in current_tag_data["ID"].values:
            
            ## overite current data
            current_tag_data.loc[current_tag_data["ID"] == tag_ID, ["X", "Y", "r", "w", "h"]] = [tags["X"], tags["Y"], tags["r"], tags["w"], tags["h"]]
            current_tag_data.to_csv(file_path, index = False)
            print(f"Updated tag: {tag_ID}")
        if tag_ID not in current_tag_data["ID"].values:     
            
            ## create new line
            pd.DataFrame([tags]).to_csv(file_path, mode = 'a', header=False, index=False)
            print(f"Added   tag: {tag_ID}")
 
def close():
    client.disconnect()