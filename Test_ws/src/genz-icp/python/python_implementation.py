#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from px4_msgs.msg import SensorGps
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares
import tf_transformations as tf
import sys

class GenZICPOdometry(Node):
    def __init__(self):
        super().__init__('genz_icp_odometry')

        # Parameters
        self.declare_parameter('max_iterations', 10)
        self.declare_parameter('convergence_threshold', 1e-4)
        self.declare_parameter('max_correspondence_distance', 1.0)
        self.declare_parameter('kernel_param', 0.1)
        self.declare_parameter('use_gps', True)  # Enabled by default for PX4
        self.declare_parameter('gps_weight', 1.0)
        self.declare_parameter('gps_reference_altitude', 0.0)  # Reference altitude (AMSL)

        # Subscribe to point cloud
        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera/depth/points_transformed',
            self.pointcloud_callback,
            10
        )

        # Publish odometry
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)

        # For GPS integration (using PX4's SensorGps message)
        self.gps_subscription = None
        self.current_gps = None
        self.gps_reference_position = None  # To store initial GPS position
        if self.get_parameter('use_gps').value:
            self.gps_subscription = self.create_subscription(
                SensorGps,
                '/fmu/out/vehicle_gps_position',
                self.gps_callback,
                10
            )

        # Initialize state
        self.prev_pc = None
        self.pose = np.eye(4)  # Identity matrix (Initial Pose)
        self.voxel_map = VoxelHashMap()  # Simplified voxel map for correspondences

    def gps_callback(self, msg):
        """Process PX4 GPS message and convert to local ENU coordinates"""
        if not msg.fix_type >= 3:  # Need 3D fix
            self.get_logger().warn("GPS doesn't have 3D fix")
            return

        # Convert to ENU coordinates (assuming first message sets reference)
        if self.gps_reference_position is None:
            self.gps_reference_position = {
                'lat': msg.lat * 1e-7,
                'lon': msg.lon * 1e-7,
                'alt': msg.alt * 1e-3
            }
            self.get_logger().info(f"Set GPS reference position: {self.gps_reference_position}")
            return

        # Convert current GPS to ENU (simplified approximation)
        # In a real implementation, use proper geodesy library like geographiclib
        lat = msg.lat * 1e-7
        lon = msg.lon * 1e-7
        alt = msg.alt * 1e-3

        # Simple approximation (for small distances)
        earth_radius = 6378137.0  # meters
        d_lat = lat - self.gps_reference_position['lat']
        d_lon = lon - self.gps_reference_position['lon']
        d_alt = alt - self.gps_reference_position['alt'] - self.get_parameter('gps_reference_altitude').value

        x = d_lon * (np.pi/180.0) * earth_radius * np.cos(self.gps_reference_position['lat'] * np.pi/180.0)
        y = d_lat * (np.pi/180.0) * earth_radius
        z = d_alt

        # Store as transformation matrix (only translation, identity rotation)
        self.current_gps = np.eye(4)
        self.current_gps[:3, 3] = [x, y, z]

    def pointcloud_callback(self, msg):
        """Process incoming point cloud with Gen-Z ICP"""
        # Extract points from message
        points = np.array([p[:3] for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)])

        if self.prev_pc is not None:
            # Run Gen-Z ICP registration
            reg = Registration(
                max_num_iterations=self.get_parameter('max_iterations').value,
                convergence_criterion=self.get_parameter('convergence_threshold').value
            )

            gps_measurement = self.current_gps if (self.get_parameter('use_gps').value and 
                                                 self.current_gps is not None) else None
            gps_weight = self.get_parameter('gps_weight').value

            final_T, _, _ = reg.register_frame(
                points, 
                self.voxel_map, 
                np.eye(4),  # We track pose separately
                self.get_parameter('max_correspondence_distance').value,
                self.get_parameter('kernel_param').value,
                gps_measurement,
                gps_weight
            )

            # Update pose (T is the transformation from current to previous frame)
            self.pose = self.pose @ np.linalg.inv(final_T)

            # Publish odometry
            self.publish_odometry()

        # Update previous point cloud (simple approach - in real implementation use voxel map)
        self.prev_pc = points

    def publish_odometry(self):
        """Publish current pose as odometry message"""
        # Extract translation and rotation
        translation = self.pose[:3, 3]
        rotation = tf.quaternions.mat_to_quat(self.pose[:3, :3])

        # Create and publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "map"
        odom_msg.child_frame_id = "base_link"

        odom_msg.pose.pose.position.x = translation[0]
        odom_msg.pose.pose.position.y = translation[1]
        odom_msg.pose.pose.position.z = translation[2]

        odom_msg.pose.pose.orientation.x = rotation[0]
        odom_msg.pose.pose.orientation.y = rotation[1]
        odom_msg.pose.pose.orientation.z = rotation[2]
        odom_msg.pose.pose.orientation.w = rotation[3]

        self.odom_publisher.publish(odom_msg)
        self.get_logger().info(f"Published odometry: {translation}", throttle_duration_sec=1.0)

# ---------------------------
# Gen-Z ICP Implementation 
# ---------------------------
def se3_exp(xi):
    """Exponential map from se(3) to SE(3)"""
    rot = Rotation.from_rotvec(xi[:3])
    T = np.eye(4)
    T[:3, :3] = rot.as_matrix()
    T[:3, 3] = xi[3:]
    return T

def transform_points(T, points):
    """Transform 3D points using transformation matrix"""
    R = T[:3, :3]
    t = T[:3, 3]
    return (R @ points.T).T + t

class VoxelHashMap:
    """Simplified voxel hash map for correspondences"""
    def __init__(self):
        self.points = None
    
    def empty(self):
        return self.points is None
    
    def get_correspondences(self, source, max_dist):
        """For simplicity, use previous frame as target"""
        if self.points is None or len(self.points) == 0:
            return (np.empty((0,3)), np.empty((0,3)), np.empty((0,3)),
                    np.empty((0,3)), np.empty((0,3)), 0, 0)
        
        # For demo: split into planar/non-planar points
        N = len(source)
        half = N // 2
        
        # Planar points (assume z=0 plane)
        src_planar = source[:half]
        tgt_planar = self.points[:half]
        normals = np.tile([0, 0, 1], (half, 1))
        
        # Non-planar points
        src_non_planar = source[half:]
        tgt_non_planar = self.points[half:]
        
        return (src_planar, tgt_planar, normals, 
                src_non_planar, tgt_non_planar, 
                half, N-half)

    def update(self, points):
        """Update the map with new points"""
        self.points = points

def cost_function(params, src_planar, tgt_planar, normals, src_non_planar, tgt_non_planar, 
                 kernel, alpha, gps_measurement=None, gps_weight=1.0):
    """Combined residual function for Gen-Z ICP"""
    T = se3_exp(params)
    residuals = []

    # Point-to-plane residuals
    if len(src_planar) > 0:
        transformed = transform_points(T, src_planar)
        res_plane = np.einsum('ij,ij->i', transformed - tgt_planar, normals)
        res_plane = res_plane / (1.0 + kernel * res_plane**2)
        residuals.append(alpha * res_plane)
    
    # Point-to-point residuals
    if len(src_non_planar) > 0:
        transformed = transform_points(T, src_non_planar)
        res_ptp = (transformed - tgt_non_planar).reshape(-1)
        res_ptp = res_ptp / (1.0 + kernel * res_ptp**2)
        residuals.append((1 - alpha) * res_ptp)
    
    # GPS constraint
    if gps_measurement is not None:
        gps_trans = gps_measurement[:3, 3]
        T_trans = T[:3, 3]
        res_gps = gps_weight * (T_trans - gps_trans)
        residuals.append(res_gps)
    
    return np.concatenate(residuals) if residuals else np.array([])

class Registration:
    def __init__(self, max_num_iterations=10, convergence_criterion=1e-4):
        self.max_num_iterations = max_num_iterations
        self.convergence_criterion = convergence_criterion

    def register_frame(self, frame, voxel_map, initial_guess, max_dist, kernel, 
                      gps_measurement=None, gps_weight=1.0):
        """Register a new frame to the map"""
        source = np.copy(frame)
        source = transform_points(initial_guess, source)
        T_icp = np.eye(4)
        
        for j in range(self.max_num_iterations):
            # Get correspondences
            src_planar, tgt_planar, normals, src_non_planar, tgt_non_planar, planar_count, non_planar_count = \
                voxel_map.get_correspondences(source, max_dist)
            
            total_count = planar_count + non_planar_count
            alpha = planar_count / total_count if total_count > 0 else 0.5
            
            # Optimize
            x0 = np.zeros(6)
            res_fun = lambda params: cost_function(
                params, src_planar, tgt_planar, normals,
                src_non_planar, tgt_non_planar,
                kernel, alpha, gps_measurement, gps_weight
            )
            result = least_squares(res_fun, x0, verbose=0, max_nfev=20)
            estimation = se3_exp(result.x)
            
            # Update
            source = transform_points(estimation, source)
            T_icp = estimation @ T_icp
            
            # Check convergence
            if np.linalg.norm(result.x) < self.convergence_criterion:
                break

        # Update voxel map (in a real implementation, this would be more sophisticated)
        voxel_map.update(frame)
        
        return T_icp @ initial_guess, None, None

def main(args=None):
    rclpy.init(args=args)
    node = GenZICPOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()