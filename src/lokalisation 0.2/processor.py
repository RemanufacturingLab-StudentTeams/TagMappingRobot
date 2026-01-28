# processor.py
import threading
import queue
import json
import tag_localistation as tla
import paho.mqtt.client as mqtt
import time
from datetime import datetime
import os
import pandas as pd


class Processor(threading.Thread):
    def __init__(self, work_queue: queue.Queue,
                 batch_count=5, mqtt_config=None, interactive=None,lokalisation_config=None):
        super().__init__(daemon=True)
        self.q = work_queue
        self.batch_count = batch_count
        assert self.batch_count > 1
        self._stop = threading.Event()
        self.mqtt_config = mqtt_config
        self.interactive = interactive
        tla.init_globals(lokalisation_config)
        
        # setup mqtt if enabled
        if (self.mqtt_config.get('enabled')):
            try:
                print ("connecting")
                self.mqtt_client = mqtt.Client(client_id = self.mqtt_config.get('mqtt_name'), callback_api_version=mqtt.CallbackAPIVersion.VERSION1  )
                self.mqtt_client.connect(self.mqtt_config.get('broker_host'), self.mqtt_config.get('broker_port'), 60)
                self.mqtt_client.loop_start()
            except Exception as e:
                print (f"error connecting to broker:{e} ")                
        
    def stop(self):
        """close mqtt connection and stop proces loop"""
        if self.mqtt_config.get('enabled'):
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
            print ("mqtt HMI: disconnected.")
        self._stop.set()
        print ("processor: Stopped")
        
    def save(self, data, name):
        """
        Parameters
        ----------
        data : dict
            data to be saved.
        name : string
            name of file.
        -------
        Saves data to a xlsx file
        """
        timestamp = datetime.now().strftime('%d%m%y_%H%M%S')
        filename = f'{name}{timestamp}.xlsx'
        
        directory = self.interactive.get('write_folder')
        os.makedirs(directory, exist_ok=True)
        filepath = os.path.join(directory, filename)

        df = pd.DataFrame(data)

        with pd.ExcelWriter(filepath, engine='openpyxl') as writer:
            df.to_excel(writer, sheet_name='All Data', index=False)
        print(f'Results saved to {filename}')

    def _publish_batch(self, results):
        """
        Parameters
        ----------
        results : dict with keys ID,X,Y,r,w,h 
        -------
        publishes data to mqtt with HMI topic

        """
        payload = json.dumps(results)
        topic = self.mqtt_config.get('topic')

        if self.mqtt_config.get('enabled'):
            try:
                self.mqtt_client.publish(topic, payload, qos=1)
                #self.mqtt_client.publish(topic, payload)
                print (f"sending 1 : {payload}")
            except Exception as e:
                print("Processor: MQTT publish error:", e)

    def run(self):
        """gets data from queue and processes when enough data is present"""
        cycles = []
        while not self._stop.is_set():
            try:
                cycle = self.q.get(timeout=0.5)
                cycles.append(cycle)
            except queue.Empty:
                continue
            if len(cycles) >= self.batch_count:
                start = time.time()
                self._process_batch(cycles)
                end = time.time()
                print (f"procesing took {end - start}seconds")
                cycles = []
        if cycles:
            print("Processor: flushing remaining cycles before shutdown")
            self._process_batch(cycles)
        
        
    def _process_batch(self, cycles):
        """" groups data per tag ID and proceses it """
        # cycles is a list of 5 measurement cycles
        grouped = {}   # tag_id -> list of measurement dicts
        for cycle in cycles:
            for m in cycle["cycle"]:
                tag = m.get("Tag ID")
                if not tag:
                    continue
                grouped.setdefault(tag, []).append(m)
        results = []
        for tag, measurements in grouped.items():
            try:
                res = tla.process_measurements(tag, measurements)
                if res:
                    results.append(res)
            except Exception as e:
                print("Localisation error for tag:", tag, e)

        if results:
            #print ("sending results", results)
            self._publish_batch(results)
        if self.interactive.get('write_to_excel'):
            data = []
            if results:
                self.save(results, self.interactive.get('result_file'))
            for cycle in cycles:
                for m in cycle["cycle"]:
                    data.append(m)
            self.save(data, self.interactive.get('RSSI_file'))
            print("Processor: finished batch of", len(results), "tags")
