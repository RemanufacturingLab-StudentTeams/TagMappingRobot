# Config.json
### used to set varibles and disable or enable functions

### Features


- batch_count: amount of measurments needed before processing (needs to be 1 or greater)

- max_size: max amount of measurments allowed in the queue to prevent overflows (same size as batch_count or greater)

- measurement_interval: delay between measurments in seconds 

- rfid_port: port where the rfid reader is connected 

- antenna_number: port where the antenna is connected to the rfid reader (1-4)

- antenna_power: (0 - 3000) use 3000 for optimal preformance

- Z_value: unsed for now can be made dynamicly later to implamnet z axis detection ( in meters)

mqtt:{
- ENABLE: enable or diable HMI connection (false or true)
- BROKER_HOST: host ip adress
- BROKER_PORT: broker port
- TOPIC: needs to be the same as the HMI topic or other recieving device 


mecabot:{
- ENABLED: enable or diable mecabot connection (false or true) if diabled defaults to manual input 
- BROKER: broker ip adress
- TOPIC: needs to be the same as mecabot topic or other sending device 


lokalisation_config:{
- DATABASE_FILE: file name of the database
- MIN_POLY_AREA: min size of poly area to prevent unrealistic small shapes (in meters, needs to be greater than 0)
- POLY_SHAPE_RATIO: ratio between area and perimter to prevent wierd shapes
- MIN_RSSI: minimal rssi value to prevent too small rssi values 
- MAX_RSSI: maximal rssi value to prevent too small rssi values 
- DEFAULT_ELLIPSE: used ellips when faild to calculate (deg, x[m], y[m])
- MAX_VARIANCE_TRESHOLD: max amount of variance between rssi values measured from one location
- OVERLAP_METHOD: 0 for hard overlap, 1 for soft voting overlap. ( Method 1 is a later implemneted function, it is therfore not mention in the paper. It handels stiuations with more noise better and is recomended to use. Instead of keeping only the area where all measurements agree, the area where most measurements agree gets kept.)


interactive:{
- write_to_excel: output results and measruemtns to xlsx file (false or true)
- write_folder: folder name to write to
- RSSI_file  : file name of measurments
- result_file: file name of the results 

