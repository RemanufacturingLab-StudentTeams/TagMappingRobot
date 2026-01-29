import threading
import json
import paho.mqtt.client as mqtt

class MQTTPoseReceiver:
    def __init__(self, broker="localhost", topic="robot/pose", port=1883, qos=1):
        self.broker = broker
        self.topic = topic
        self.port = port
        self.qos = qos

        self._latest_pose = None
        self._lock = threading.Lock()

        self.client = mqtt.Client( callback_api_version=mqtt.CallbackAPIVersion.VERSION1  )
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message

    def start(self):
        """Connects and starts the MQTT client in the background."""
        self.client.connect(self.broker, self.port, 60)
        self.client.loop_start()

    def stop(self):
        """Stops the MQTT client cleanly."""
        self.client.loop_stop()
        self.client.disconnect()

    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            client.subscribe(self.topic, qos=self.qos)
            print(f"[MQTT] Connected to {self.broker}, subscribed to {self.topic}")
        else:
            print(f"[MQTT] Connection failed with code {rc}")

    def _on_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode())
            with self._lock:
                self._latest_pose = data  # Expecting {"x":..., "y":..., "yaw":...}
        except Exception as e:
            print("MQTT decode error:", e)

    def get_pose(self):
        """Return the latest pose as (x, y, yaw) in meters and degrees or None if not available."""
        with self._lock:
            if self._latest_pose:
                return (
                    self._latest_pose.get("x"),
                    self._latest_pose.get("y"),
                    self._latest_pose.get("yaw")
                )
            return None
