import os
import pandas as pd
from config import NAME_FILE

def ensure_name_file_exist():
    """Create the tag_names.csv if it doesn't exist"""
    if not os.path.exists(NAME_FILE):
        pd.DataFrame(columns=["ID", "name"]).to_csv(NAME_FILE, index=False)

def load_names():
    """Load tag names CSV as DataFrame"""
    ensure_name_file_exist()
    names = pd.read_csv(NAME_FILE)
    names["ID"] = names["ID"].astype(str)
    return names

def append_name(tag_id: str, new_name: str):
    """Append a new (ID, name) row to tag_names.csv"""
    names = load_names()
    names.loc[len(names)] = {"ID": str(tag_id), "name": new_name}
    names.to_csv(NAME_FILE, index=False)