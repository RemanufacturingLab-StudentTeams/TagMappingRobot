# mqtt_listner.py
### used to recive mqtt data from mecabot
### Features
- setup connection
- extract data 

### Functions
def __init__(self, broker="localhost", topic="robot/pose", name="runner", port=1883, qos=1):
- sets varibles 
def start(self):
- starts connection
def stop(self):
- safley closes connection
def _on_connect(self, client, userdata, flags, rc):
- verfy connection
def _on_message(self, client, userdata, msg):
- when a message is recived, set it to latest_pose
def get_pose(self):
- returns _latest_pose