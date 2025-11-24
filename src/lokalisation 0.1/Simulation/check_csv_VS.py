import pandas as pd
import os

def init_csv(WRITE_FOLDER, OUTPUT_FILE):
    """Create the main CSV file if it does not exist."""
    file_path = os.path.join(WRITE_FOLDER, OUTPUT_FILE)
    if not os.path.exists(file_path):
        pd.DataFrame(columns=["ID", "X", "Y", "r", "w", "h"]).to_csv(file_path, index=False)
        print(f"Created new file: {OUTPUT_FILE}\nat: {file_path}")    
        

def is_file_ready(file_path, retries=3):
    """Check if a file is ready to be read (not being written)."""
    for _ in range(retries):
        try:
            with open(file_path, "rb"):
                return True
        except PermissionError:
            return False
    return False


def update_tag_data(tag_data, file_path):
    ## read current data file
    current_tag_data = pd.read_csv(file_path)

    ##check for every ID 
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

    