#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
from px4_msgs.msg import SensorGps
import gtsam
from gtsam.symbol_shorthand import X
from scipy.spatial import cKDTree

class FactorGraphSLAM(Node):
    def __init__(self):
        super().__init__('factor_graph_slam')
        
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.graph = gtsam.NonlinearFactorGraph()
        self.initial_estimates = gtsam.Values()
        self.current_pose_key = 0
        self.last_pose = None
        
        self.global_map = []
        self.map_poses = []
        
        self.icp_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.1, 0.05, 0.05, 0.05]))
        self.gps_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([1.0, 1.0, 1.0, 0.1, 0.1, 0.1]))
        self.point_to_point_noise = gtsam.noiseModel.Isotropic.Sigma(3, 0.1)
        
        self.map_pub = self.create_publisher(PointCloud2, '/slam_map', map_qos)
        
        self.points_sub = self.create_subscription(
            PointCloud2,
            '/camera/points_transformed',
            self.points_callback,
            sensor_qos)
        self.gps_sub = self.create_subscription(
            SensorGps,
            '/fmu/out/vehicle_gps_position',
            self.gps_callback,
            sensor_qos)
        
        self.previous_points = None
        self.current_points = None
        
        self.get_logger().info("FactorGraphSLAM node initialized")

    def points_callback(self, msg):
        self.get_logger().info(f"Received point cloud with {msg.width * msg.height} points")
        points_list = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        points = np.array([[p[0], p[1], p[2]] for p in points_list], dtype=np.float32)
        
        self.current_points = points[::10]
        self.get_logger().info(f"Processed {len(self.current_points)} valid points")
        
        if self.previous_points is not None and self.current_points.size > 0:
            self.get_logger().info("Running ICP and updating graph")
            relative_pose = self.compute_icp_gtsam(self.previous_points, self.current_points)
            
            if self.last_pose is None:
                self.last_pose = gtsam.Pose3()
                self.initial_estimates.insert(X(0), self.last_pose)
                self.map_poses.append(self.last_pose)
                self.global_map.append(self.current_points)
                self.get_logger().info("Initialized first pose with point cloud")
            else:
                current_pose_key = self.current_pose_key + 1
                self.graph.add(gtsam.BetweenFactorPose3(
                    X(self.current_pose_key), 
                    X(current_pose_key), 
                    relative_pose, 
                    self.icp_noise
                ))
                
                self.last_pose = self.last_pose.compose(relative_pose)
                self.initial_estimates.insert(X(current_pose_key), self.last_pose)
                self.current_pose_key = current_pose_key
                self.map_poses.append(self.last_pose)
                self.global_map.append(self.current_points)
                self.get_logger().info(f"Added pose {self.current_pose_key}, map size: {len(self.global_map)}")
                
                self.optimize_graph()
                self.publish_map()
        
        self.previous_points = self.current_points.copy()

    def gps_callback(self, msg):
        self.get_logger().info("Received GPS data")
        gps_position = gtsam.Point3(msg.latitude_deg / 1e7, msg.longitude_deg / 1e7, msg.altitude_msl_m / 1000.0)
        gps_pose = gtsam.Pose3(gtsam.Rot3(), gps_position)
        
        if self.current_pose_key == 0 and self.last_pose is None:
            self.last_pose = gps_pose
            self.initial_estimates.insert(X(0), self.last_pose)
            self.map_poses.append(self.last_pose)
            self.get_logger().info("Initialized first pose with GPS data")
        
        if self.current_pose_key >= 0 and self.last_pose is not None:
            gps_pose = gtsam.Pose3(self.last_pose.rotation(), gps_position)
            self.graph.add(gtsam.PriorFactorPose3(
                X(self.current_pose_key),
                gps_pose,
                self.gps_noise
            ))
            self.optimize_graph()
            if self.global_map:
                self.publish_map()

    def compute_icp_gtsam(self, source_points, target_points):
        tree = cKDTree(target_points)
        distances, indices = tree.query(source_points, k=1)
        tgt_points = target_points[indices]

        icp_graph = gtsam.NonlinearFactorGraph()
        icp_initial = gtsam.Values()
        pose_symbol = X(0)
        icp_initial.insert(pose_symbol, gtsam.Pose3())

        # Create expression for pose3
        pose_expr = gtsam.ExpressionPose3(pose_symbol)

        for src, tgt in zip(source_points, tgt_points):
            src_point = gtsam.Point3(*src)
            tgt_point = gtsam.Point3(*tgt)
            # Transform source point using the pose expression
            transformed_expr = gtsam.ExpressionPoint3(pose_expr.transformFrom(src_point))
            # Add error factor: transformed source should match target
            factor = gtsam.ExpressionFactorPoint3(
                self.point_to_point_noise,
                tgt_point,
                transformed_expr
            )
            icp_graph.add(factor)

        params = gtsam.LevenbergMarquardtParams()
        params.setMaxIterations(10)
        optimizer = gtsam.LevenbergMarquardtOptimizer(icp_graph, icp_initial, params)
        result = optimizer.optimize()
        
        self.get_logger().info("ICP completed")
        return result.atPose3(pose_symbol)

    def optimize_graph(self):
        optimizer = gtsam.LevenbergMarquardtOptimizer(self.graph, self.initial_estimates)
        result = optimizer.optimize()
        
        self.initial_estimates = result
        for i in range(len(self.map_poses)):
            self.map_poses[i] = result.atPose3(X(i))
        
        current_pose = result.atPose3(X(self.current_pose_key))
        self.get_logger().info(f"Current Pose: {current_pose.translation()}")

    def publish_map(self):
        if not self.global_map or not self.map_poses:
            self.get_logger().warn("No map data to publish: global_map or map_poses is empty")
            return

        self.get_logger().info(f"Publishing map with {len(self.global_map)} point clouds")
        global_points = []
        for pose, points in zip(self.map_poses, self.global_map):
            rot = pose.rotation().matrix()
            trans = pose.translation()
            transformed_points = np.dot(points, rot.T) + trans
            global_points.append(transformed_points)
        
        global_points = np.vstack(global_points)
        self.get_logger().info(f"Total points in map: {len(global_points)}")

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"
        
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        cloud = pc2.create_cloud(header, fields, global_points)
        self.map_pub.publish(cloud)
        self.get_logger().info("Map published to /slam_map")

def main(args=None):
    rclpy.init(args=args)
    slam = FactorGraphSLAM()
    rclpy.spin(slam)
    slam.destroy_node()
    rclpy.shutdown()
