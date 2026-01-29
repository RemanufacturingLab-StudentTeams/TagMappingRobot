#!/usr/bin/env python3
"""
This file gets the current location of the Mecabot and sends it to a MQTT broker.
This data is used by the localasation algorithm to calculat tag location.
"""
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener
import paho.mqtt.client as mqtt
import json
from tf_transformations import euler_from_quaternion
import numpy as np


broker = "localhost"
topic = "test/topic"
delaytijd = 0.5 #seconden

client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION1)
client.connect(broker)

class MapTFListener(Node):
    def __init__(self):
        super().__init__('map_tf_listener')
        self.get_logger().info("Node started!")

        #TF Mecabot X
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        #positie (x,y,Rz)
        self.x = None
        self.y = None
        self.yaw = None

        #MQTT
        self.client = mqtt.Client(
            callback_api_version=mqtt.CallbackAPIVersion.VERSION1
        )
        self.client.connect(broker)
        self.client.loop_start()

        #timers
        self.create_timer(1.0, self.lookup_position)      # TF lookup
        self.create_timer(delaytijd, self.publish_data)   # MQTT publish


    def lookup_position(self):
        self.get_logger().info("Attempting TF lookup...")
        try:
            trans = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )
            #positie (X,Y)
            self.x = trans.transform.translation.x
            self.y = trans.transform.translation.y

            #rotatie richting in rad (om de Z as)
            qx = trans.transform.rotation.x
            qy = trans.transform.rotation.y
            qz = trans.transform.rotation.z
            qw = trans.transform.rotation.w

            roll, pitch, yaw = euler_from_quaternion([qx, qy, qz, qw])
            self.yaw = yaw
            self.yaw_degree = np.rad2deg(yaw) #radialen naar graden

            self.get_logger().info(f"Position: x={self.x:.2f}, y={self.y:.2f}, yaw={self.yaw:.2f}")

        except Exception as e:
            self.get_logger().warn(f"TF error: {e}")

    def publish_data(self):
        if self.x is None:
            return

        msg = json.dumps({
            "x": self.x,
            "y": self.y,
            "yaw": self.yaw_degree
        })

        self.client.publish(topic, msg)

def main():
    rclpy.init()
    node = MapTFListener()

    try:
        rclpy.spin(node)
    finally:
        node.client.loop_stop()
        node.client.disconnect()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
