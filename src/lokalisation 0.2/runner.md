# runner.py
### this is main runner used to strat the localisation, communicatian and data extraction. uses the config file to set all varibles

### Features
- initilaize all files
- load config file
- call data exrtaction function every set intervall
- add data to proces queue
- cals function to get current posisiton

### Functions
def load_config()
- loads the congfig file

def antenna_pos_interactive()
- get manual antenna location via terminal (used for testing)

def tag_pos_interactive(tag_id)
- get manual tag location for tag_id via terminal (used for testing)

def init(cfg):
- initilaze classes and files with values from cfg

def main()
- calls al setup functions and loops every intervall to get measurment postion and antenna data