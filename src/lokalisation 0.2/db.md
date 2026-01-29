# DB.py
### creates and manages a database file where previos calculated polygons are saved.

### Features
- Create file if it does not exist
- Save polygon for tag ID
- Get polygon for tag ID

### Functions
def init_db(File_name)
- creates a database with the name given to 'File_name'

def save_polygon(tag_id, polygon)
- save polyong with tag_id

def get_polygon(tag_id)
- returns polygon saved under 'tag_id'