import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanFilter(Node):
    def __init__(self):
        super().__init__("scan_filter")

        # Blindspot (in radians) – pas aan wat je wilt
        self.blind_start = math.radians(160)  # beginhoek blindspot
        self.blind_end = math.radians(240)  # eindhoek blindspot

        # Abonneer op de ruwe LiDAR data
        self.subscription = self.create_subscription(
            LaserScan, "/scan", self.callback, 10
        )

        # Publiceer de gefilterde scan
        self.publisher_ = self.create_publisher(LaserScan, "/scanned", 10)

        self.get_logger().info(
            f"ScanFilter actief — blindspot van {math.degrees(self.blind_start):.1f}° "
            f"tot {math.degrees(self.blind_end):.1f}°"
        )

    def callback(self, msg: LaserScan):
        self.get_logger().info("Callback funciton started!")
        filtered = LaserScan()
        filtered.header = msg.header
        filtered.angle_min = msg.angle_min
        filtered.angle_max = msg.angle_max
        filtered.angle_increment = msg.angle_increment
        filtered.time_increment = msg.time_increment
        filtered.scan_time = msg.scan_time
        filtered.range_min = msg.range_min
        filtered.range_max = msg.range_max
        filtered.ranges = msg.ranges
        angle = msg.angle_min

        for i in range(len(msg.ranges)):  # Loop om blindspots blind te maken.
            if self.blind_start <= angle <= self.blind_end:
                filtered.ranges[i] = float("inf")  # niets gezien
            angle += msg.angle_increment

        self.publisher_.publish(filtered)


def main(args=None):
    rclpy.init(args=args)
    node = ScanFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":  # zorgt voor opstarten
    main()
