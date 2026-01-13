import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
import paho.mqtt.client as mqtt
import json
from tf_transformations import euler_from_quaternion
import numpy as np


broker = "localhost"
topic = "test/topic"

client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION1)
client.connect(broker)

class MapTFListener(Node):
    def __init__(self):
        super().__init__('map_tf_listener')
        self.get_logger().info("Node started!")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.x = None
        self.y = None
        self.yaw = None
        
        self.timer = self.create_timer(1.0, self.lookup_position)

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
            self.yaw_degree = np.rad2deg(yaw)

            self.get_logger().info(f"Position: x={self.x:.2f}, y={self.y:.2f}, yaw={self.yaw:.2f}")
        
        except Exception as e:
            self.get_logger().warn(f"TF error: {e}")

def main():
    rclpy.init()
    node = MapTFListener()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=1.0)

            if node.x is not None:
                msg = json.dumps({"x": node.x, "y": node.y, "yaw": node.yaw_degree})
                client.publish(topic, msg)
    finally:
        client.disconnect()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
