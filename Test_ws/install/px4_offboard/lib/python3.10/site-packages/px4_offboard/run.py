import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleStatus, VehicleLocalPosition, VehicleCommand
import numpy as np
import math

class OffboardControl(Node):
    def __init__(self):
        super().__init__('offboard_control')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        
        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)

        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Home GPS location (origin of NED)
        self.lat0 = 37.413723
        self.lon0 = -121.997299

        gps_waypoints = [
            (37.413723, -121.997299),
            (37.413822, -121.997571),
            (37.414070, -121.997359),
            (37.413991, -121.997135),
        ]

        # Convert GPS to NED: [North, East, Down]
        self.waypoints = [self.gps_to_ned(lat, lon) + [15.0] for lat, lon in gps_waypoints]  # 15m down = -15 altitude
        self.waypoint_index = -1  # -1 means takeoff

        self.current_position = [0.0, 0.0, 0.0]  # NED
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED

        self.timer = self.create_timer(0.1, self.send_commands)

    def gps_to_ned(self, lat, lon):
        """Convert GPS to NED in meters relative to home"""
        R = 6378137.0  # Earth radius
        d_lat = math.radians(lat - self.lat0)
        d_lon = math.radians(lon - self.lon0)
        north = R * d_lat
        east = R * d_lon * math.cos(math.radians((lat + self.lat0) / 2))
        return [north, east]  # NED: x=North, y=East

    def vehicle_status_callback(self, msg):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def vehicle_local_position_callback(self, msg):
        self.current_position = [msg.y, msg.x, -msg.z]  # Convert PX4's local ENU to NED

        if self.waypoint_index == -1:
            if abs(msg.z + 15.0) < 0.3:
                self.get_logger().info("Takeoff complete. Navigating to waypoints.")
                self.waypoint_index = 0
        elif self.waypoint_index < len(self.waypoints):
            target = self.waypoints[self.waypoint_index]
            if (abs(self.current_position[0] - target[0]) < 0.5 and
                abs(self.current_position[1] - target[1]) < 0.5 and
                abs(self.current_position[2] - target[2]) < 0.4):
                self.waypoint_index += 1
                if self.waypoint_index >= len(self.waypoints):
                    self.get_logger().info("Final waypoint reached. Landing.")
                    self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                else:
                    self.get_logger().info(f"Moving to next waypoint: {self.waypoints[self.waypoint_index]}")

    def send_commands(self):
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position = True
        self.offboard_control_mode_publisher.publish(offboard_msg)

        if self.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
            self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            self.get_logger().info("Sent arm command.")
            return

        if self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
            self.get_logger().info("Sent Offboard mode command.")
            return

        traj = TrajectorySetpoint()
        traj.timestamp = int(Clock().now().nanoseconds / 1000)

        if self.waypoint_index == -1:
            traj.position = np.array([0.0, 0.0, 15.0], dtype=np.float32)  # 15m down = -15m altitude
            traj.yaw = 0.0
            self.get_logger().info("Taking off to 15 meters (Down).")
        elif self.waypoint_index < len(self.waypoints):
            target = self.waypoints[self.waypoint_index]
            traj.position = np.array(target, dtype=np.float32)

            # Yaw angle to face next waypoint
            dx = target[0] - self.current_position[0]
            dy = target[1] - self.current_position[1]
            traj.yaw = math.atan2(dy, dx)
        else:
            return

        self.trajectory_setpoint_publisher.publish(traj)

    def send_vehicle_command(self, command, param1=0.0, param2=0.0):
        cmd = VehicleCommand()
        cmd.timestamp = int(Clock().now().nanoseconds / 1000)
        cmd.command = command
        cmd.param1 = param1
        cmd.param2 = param2
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        self.vehicle_command_publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = OffboardControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
