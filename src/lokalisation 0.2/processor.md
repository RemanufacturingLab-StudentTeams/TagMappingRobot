# processor.py
### gets data from the queue and proceses it when it has enough, results get communicated with the HMI or saved to excel.

### Features
- start communication
- save data
- publish data
- sort data
- gets data from queue

### Functions

 def __init__(self, work_queue, batch_count, mqtt_config, interactive, lokalisation_config)
 - starts communication and sets varibles 

 def stop(self):
 - stops proces loop and communication

 def save (self, data, name):
 - saves data in a .xlsx file under name and a timestamp

 def _publish_batch(self, results):
 - publeshes resluts to mqtt with predefined topic 

 def run(self):
 - runs in loop while _stop.is_set() extracts data from queue and if enough data is extracted hand it to _proces_batch

 def _process_batch(self, cycles):
 - sorts all data for "tag ID", gives data per tag to process_measurment and saves results and measurments to xlsx if write_to_excel is true