import pandas as pd
import time
        
def is_file_ready(file_path, retries=3, delay=1):
    """Check if a file is ready to be read (not being written)."""
    for _ in range(retries):
        try:
            pd.read_excel(file_path, nrows=1)
            return True
        except Exception:
            time.sleep(delay)
    return False


def update_tagg_data(tag_data, file_path):
    ## read current data file
    current_tag_data = pd.read_csv(file_path)

    ##check for every ID 
    for tags in tag_data:  
        tag_ID = tags["ID"]
        
        if tag_ID in current_tag_data["ID"].values:
            
            ## overite current data
            current_tag_data.loc[current_tag_data["ID"] == tag_ID, ["X", "Y", "r"]] = [tags["X"], tags["Y"], tags["r"]]
            current_tag_data.to_csv(file_path, index = False)
            print(f"Updated existing tag: {tag_ID}")
        if tag_ID not in current_tag_data["ID"].values:     
            
            ## create new line
            pd.DataFrame([tags]).to_csv(file_path, mode = 'a', header=False, index=False)
            print(f"Added new tag: {tag_ID}")
            
            
#def save_common_intersection(common_intersection, file_path):
    