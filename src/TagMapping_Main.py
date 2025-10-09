import tagg_localistation_WV as taggLoc
import os
import glob
import time
import pandas as pd
import check_csv as check

folder = r'C:\Users\yoeri\De Haagse Hogeschool\Tag mapping robot_groups - Data'
file  = "data1.csv"



if __name__ == "__main__":
    start = time.perf_counter()
    
    #Look for excel file with that name. 
    excel_files = glob.glob('rfid_data_140525_132422-Test6.xlsx')
    if not excel_files: 
        print("No RFID data files found in the current directory!")
        exit(1)
    
    ## Gets the newest file, handig voor onze toepassing?
    latest_file = max(excel_files, key=lambda x: os.path.getmtime(x))
    print(f"Using most recent data file: {latest_file}")
    ## Start process
    
    try:
        ## set path dir
        file_path = os.path.join(folder, file)
        
        ## if file does not exist, creat it
        if not os.path.exists(file_path):
            pd.DataFrame(columns=["ID", "X","Y", "r"]).to_csv(file_path, index=False)
            print(f"Created new file: {file}\nat: {file_path}")
        print (f"{'-'*30}")    
        
        ## process tagg data and write results
        tag_data = taggLoc.process_tag_data(latest_file, file_path)
        check.update_tagg_data(tag_data, file_path)
        
        
        eind = time.perf_counter()
        full_code_time = round(eind-start,4)
        print(f"{'-'*30}\nFull code time: {full_code_time}")

    except KeyboardInterrupt:
        print("\nProcessing stopped by user.")
    except Exception as e:
        print(f"\nAn error occurred: {str(e)}")