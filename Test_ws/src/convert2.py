#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from px4_msgs.msg import SensorGps
from sensor_msgs.msg import NavSatFix

class Px4GpsConverter(Node):
    def __init__(self):
        super().__init__('px4_gps_converter')

        # Define QoS profile to match PX4 (Best Effort, Volatile)
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        # Subscriber for /px4_1/fmu/out/vehicle_gps_position
        self.subscription = self.create_subscription(
            SensorGps,
            '/px4_1/fmu/out/vehicle_gps_position',
            self.callback,
            qos_profile
        )

        # Publisher for /gps/fix
        self.publisher = self.create_publisher(NavSatFix, '/gps/fix', 10)

    def callback(self, msg):
        gps_msg = NavSatFix()
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        gps_msg.header.frame_id = 'base_link'  # Adjust if needed

        # Latitude, longitude, and altitude
        gps_msg.latitude = msg.latitude_deg
        gps_msg.longitude = msg.longitude_deg
        gps_msg.altitude = msg.altitude_msl_m

        # Position covariance using eph (horizontal) and epv (vertical)
        gps_msg.position_covariance = [
            msg.eph**2, 0.0, 0.0,
            0.0, msg.eph**2, 0.0,
            0.0, 0.0, msg.epv**2
        ]
        gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        self.publisher.publish(gps_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Px4GpsConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
