# processor.py
import threading
import queue
import json
import tag_localistation as tla
import paho.mqtt.client as mqtt
import time

class Processor(threading.Thread):
    def __init__(self, work_queue: queue.Queue,
                 batch_count=5, batch_interval=5.0, mqtt_config=None):
        super().__init__(daemon=True)
        self.q = work_queue
        self.batch_count = batch_count
        self.batch_interval = batch_interval
        self._stop = threading.Event()
        self.mqtt_config = mqtt_config or {}
        
        # setup mqtt if enabled
        if self.mqtt_config.get('enabled'):
            try:
                print ("connecting")
                self.mqtt_client = mqtt.Client()
                self.mqtt_client.connect(self.mqtt_config.get('broker_host'), self.mqtt_config.get('broker_port'), 60)
            except Exception as e:
                print (f"error connecting to broker:{e} ")
                
    def stop(self):
        if self.mqtt_config.get('enabled'):
            self.mqtt_client.disconnect()
        self._stop.set()

    def _publish_batch(self, results):
        if not self.mqtt_config.get('enabled'):
            return
        payload = json.dumps(results)
        topic = self.mqtt_config.get('topic')
        try:
            self.mqtt_client.publish(topic, payload)
            #print (f"sending {payload}")
        except Exception as e:
            print("Processor: MQTT publish error:", e)

    def run(self):
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



    def _process_batch(self, cycles):
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
            self._publish_batch(results)
            print("Processor: finished batch of", len(results), "tags")
