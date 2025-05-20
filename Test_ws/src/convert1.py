#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from px4_msgs.msg import SensorCombined
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3

class Px4ImuConverter(Node):
    def __init__(self):
        super().__init__('px4_imu_converter')

        # Define QoS profile to match PX4 (Best Effort, Volatile, depth=10)
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        # Subscriber for /px4_1/fmu/out/sensor_combined with matching QoS
        self.subscription = self.create_subscription(
            SensorCombined,
            '/px4_1/fmu/out/sensor_combined',
            self.callback,
            qos_profile
        )

        # Publisher for /imu/data
        self.publisher = self.create_publisher(Imu, '/imu/data', 10)

    def callback(self, msg):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'base_link'  # Adjust to your robot's frame

        # Linear accelerations (m/s²)
        imu_msg.linear_acceleration = Vector3(
            x=float(msg.accelerometer_m_s2[0]),
            y=float(msg.accelerometer_m_s2[1]),
            z=float(msg.accelerometer_m_s2[2])
        )

        # Angular velocities (rad/s)
        imu_msg.angular_velocity = Vector3(
            x=float(msg.gyro_rad[0]),
            y=float(msg.gyro_rad[1]),
            z=float(msg.gyro_rad[2])
        )

        # No orientation data (LIO-SAM doesn't require it)
        imu_msg.orientation_covariance[0] = -1

        # Covariances (tune based on your IMU specs)
        imu_msg.linear_acceleration_covariance = [0.01] * 9  # Example
        imu_msg.angular_velocity_covariance = [0.01] * 9    # Example

        self.publisher.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Px4ImuConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
