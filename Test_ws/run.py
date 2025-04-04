import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleStatus, TrajectorySetpoint

class PX4GoTo(Node):
    def __init__(self):
        super().__init__('px4_goto_node')

        # Subscribe to vehicle status
        self.subscription = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v1',  # Updated topic name
            self.vehicle_status_callback,
            10
        )

        # Publisher for GoTo commands
        self.goto_publisher = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/goto_setpoint',
            10
        )

        self.nav_state = None
        self.arming_state = None
        self.timer = self.create_timer(0.2, self.publish_goto)  # Send command every 200ms
        self.get_logger().info("PX4 GoTo Node Initialized")

    def vehicle_status_callback(self, msg):
        """Update PX4 status (arming and mode)."""
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def publish_goto(self):
        """Continuously send GoTo commands if in OFFBOARD mode & ARMED."""
        if self.nav_state == 14 and self.arming_state == 2:  # OFFBOARD = 14, ARMED = 2
            goto_msg = TrajectorySetpoint()
            goto_msg.position = [5.0, 5.0, -2.0]  # Target Position (X=5, Y=5, Z=-2)
            goto_msg.yaw = 0.0
            self.goto_publisher.publish(goto_msg)
            self.get_logger().info("📍 Sending GoTo Command: X=5, Y=5, Z=-2")
        else:
            self.get_logger().warn("PX4 is NOT in Offboard Mode or NOT Armed!")

def main(args=None):
    rclpy.init(args=args)
    node = PX4GoTo()
    rclpy.spin(node)  # Keep node running
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

