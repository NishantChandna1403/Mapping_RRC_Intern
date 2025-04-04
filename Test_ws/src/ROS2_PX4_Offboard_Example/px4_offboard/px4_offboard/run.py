import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleStatus, VehicleLocalPosition, VehicleCommand
import numpy as np
import math

class OffboardControl(Node):
    def __init__(self):
        super().__init__('setpoint')

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
        
        self.waypoints = [
            [300.0, -1000.0, -0.4]  # North, East, Down (NED frame)
        ]
        self.waypoint_index = -1  # Start with takeoff
        
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        self.current_position = [0.0, 0.0, 0.0]  # [x, y, z] in NED
        
        self.timer = self.create_timer(0.1, self.send_commands)

    def vehicle_status_callback(self, msg):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state
        self.get_logger().info(f'NAV_STATUS: {self.nav_state}, ARMING_STATUS: {self.arming_state}')

    def vehicle_local_position_callback(self, msg):
        # Update current position
        self.current_position = [msg.x, msg.y, msg.z]
        x, y, z = msg.x, msg.y, msg.z
        self.get_logger().info(f'Received Position - X: {x}, Y: {y}, Z: {z}')
        
        if self.waypoint_index == -1:  # Takeoff phase
            if abs(z + 0.5) < 0.3:
                self.waypoint_index = 0
                self.get_logger().info("Takeoff complete. Starting waypoint navigation.")
        elif (self.waypoints[self.waypoint_index][0] - 0.5 < x < self.waypoints[self.waypoint_index][0] + 0.5 and
              self.waypoints[self.waypoint_index][1] - 0.5 < y < self.waypoints[self.waypoint_index][1] + 0.5 and
              self.waypoints[self.waypoint_index][2] - 0.3 < z < self.waypoints[self.waypoint_index][2] + 0.3):
            self.waypoint_index += 1
            if self.waypoint_index >= len(self.waypoints):
                self.get_logger().info("Reached final waypoint, landing...")
                self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                return
            self.get_logger().info(f'Next waypoint: {self.waypoints[self.waypoint_index]}')

    def send_commands(self):
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        self.offboard_control_mode_publisher.publish(offboard_msg)

        if self.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
            self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            self.get_logger().info("Sent arm command.")
            return

        if self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
            self.get_logger().info("Sent offboard mode command.")
            return

        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000)

        if self.waypoint_index == -1:
            self.get_logger().info("Taking off to 5m height.")
            trajectory_msg.position = np.array([0.0, 0.0, -0.5], dtype=np.float32)
            trajectory_msg.velocity = np.array([0.0, 0.0, -0.8], dtype=np.float32)  # Controlled ascent speed
            trajectory_msg.yaw = 0.0  # Default yaw during takeoff (facing north)
        elif self.waypoint_index < len(self.waypoints):
            target_pos = self.waypoints[self.waypoint_index]
            trajectory_msg.position = np.array(target_pos, dtype=np.float32)
            trajectory_msg.velocity = np.array([1.0, 1.0, 0.0], dtype=np.float32)  # Slowed-down navigation
            
            # Calculate yaw to face the waypoint
            delta_north = target_pos[0] - self.current_position[0]  # Δx (north)
            delta_east = target_pos[1] - self.current_position[1]   # Δy (east)
            yaw_rad = math.atan2(delta_east, delta_north)  # Yaw in radians
            trajectory_msg.yaw = float(yaw_rad)  # Set yaw in radians
        
        self.trajectory_setpoint_publisher.publish(trajectory_msg)
        self.get_logger().info(f'Sent Trajectory Setpoint: {trajectory_msg.position.tolist()}, Yaw: {trajectory_msg.yaw}')

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