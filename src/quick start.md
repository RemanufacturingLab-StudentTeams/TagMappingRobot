# How to start the lokalisation and HMI software

### connect to the Raspberry
make sure you are connected to the same network as the raspberry (iotroam). Acces the Raspberry via SSH with:
- IP: __10.35.4.205__
- login: __dongguan__
- password: __Wheeltec__

(It is recomended to open 2 termials to run both the HMI and lokalisation)

### navigate to the right folder
type __ls__ to see all files and folders in the current folder, use __cd__ to open folders and navigate to __lokalisation__ or __HMI__. Open this folder and type __source .venv/bin/activate__ to activate the virtual envoirment.

### run files
It is recomended to remove the old data bases when operating in a new pysical enviorement, type __rm tag_data_lok.db__ to remove the lokalisation database and __rm tag_data_HMI.db__ to remove the HMI database. To run the HMI type __./HMIapp.py__ and for the lokalisation type __./runner.py__

(if you want to run on a new device, fisrt create a virtual enviorment by typing __python -m venv .venv__ and use __pip install -r ./requirements.txt__ to install all the required libraries)