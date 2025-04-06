import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import numpy as np
from gtsam import NonlinearFactorGraph, Values, PriorFactorPose2, Pose2, noiseModel
from std_msgs.msg import Header
from pyproj import Transformer

class GICPGPSFusionNED(Node):
    def __init__(self):
        super().__init__('gicp_gps_fusion_ned')
        self.graph = NonlinearFactorGraph()
        self.initial_estimates = Values()
        self.pose_id = 0

        # Publishers and subscribers
        self.pose_pub = self.create_publisher(PoseStamped, '/optimized_pose_ned', 10)
        self.gicp_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/icp_odom', self.gicp_callback, 10)
        self.gps_sub = self.create_subscription(
            NavSatFix, '/gps/fix', self.gps_callback, 10)

        # NED conversion setup
        self.ned_transformer = None
        self.ref_lat = None
        self.ref_lon = None
        self.ref_alt = None
        self.gicp_offset = None  

        # Parameters
        self.w_max = 0.4  # Max GICP weight
        self.k = 500.0  # Tuning constant
        self.gps_noise = noiseModel.Diagonal.Sigmas(np.array([1.0, 1.0]))  # Base GPS noise

    def set_ned_reference(self, lat, lon, alt):
        # Set first GPS fix as NED origin
        self.ref_lat, self.ref_lon, self.ref_alt = lat, lon, alt
        self.ned_transformer = Transformer.from_crs(
            "epsg:4326",  # WGS84 lat/lon
            "+proj=tmerc +lat_0={} +lon_0={} +k=1 +x_0=0 +y_0=0 +ellps=WGS84 +units=m".format(lat, lon),
            always_xy=True
        )
        self.get_logger().info(f"NED reference set: lat={lat}, lon={lon}, alt={alt}")

    def gps_to_ned(self, lat, lon, alt):
        if self.ned_transformer is None:
            self.set_ned_reference(lat, lon, alt)
            return 0.0, 0.0  # Origin
        x, y = self.ned_transformer.transform(lon, lat)  # lon, lat order for pyproj
        z = -(alt - self.ref_alt)  # Down is positive
        return x, y  # North, East

    def gicp_callback(self, msg):
        # GICP pose in local frame
        x_local = msg.pose.pose.position.x
        y_local = msg.pose.pose.position.y
        theta = self.quat_to_yaw(msg.pose.pose.orientation)
        cov = np.array(msg.pose.covariance).reshape(6, 6)[0:3, 0:3]
        N_t = msg.covariance[0]  # Placeholder for point count

        # Convert to NED (assume initial alignment with first GPS)
        if self.gicp_offset is None and self.ref_lat is not None:
            self.gicp_offset = [x_local, y_local]  # First GICP pose aligns with NED origin
        x_ned = x_local - (self.gicp_offset[0] if self.gicp_offset else 0)
        y_ned = y_local - (self.gicp_offset[1] if self.gicp_offset else 0)

        z_gicp = Pose2(x_ned, y_ned, theta)
        self.initial_estimates.insert(self.pose_id, z_gicp)

        # Weight and factor
        w_gicp = min(self.w_max, N_t / (N_t + self.k))
        w_gps = 1.0 - w_gicp
        adjusted_cov = cov / w_gicp
        gicp_noise = noiseModel.Gaussian.Covariance(adjusted_cov)
        self.graph.add(PriorFactorPose2(self.pose_id, z_gicp, gicp_noise))
        self.get_logger().info(f"Added GICP: pose_id={self.pose_id}, N_t={N_t}, w_gicp={w_gicp}")

        self.optimize_and_publish()

    def gps_callback(self, msg):
        # GPS to NED
        x_ned, y_ned = self.gps_to_ned(msg.latitude, msg.longitude, msg.altitude)
        gps_pose = Pose2(x_ned, y_ned, 0.0)

        # Weight and factor
        w_gicp = min(self.w_max, self.initial_estimates.atPose2(self.pose_id - 1).x() / (self.k + self.initial_estimates.atPose2(self.pose_id - 1).x()))  # Placeholder N_t from last pose
        w_gps = 1.0 - w_gicp
        adjusted_gps_noise = noiseModel.Diagonal.Sigmas(np.array([1.0 / w_gps, 1.0 / w_gps]))
        self.graph.add(PriorFactorPose2(self.pose_id - 1, gps_pose, adjusted_gps_noise))
        self.get_logger().info(f"Added GPS: pose_id={self.pose_id - 1}, w_gps={w_gps}")

        self.optimize_and_publish()

    def optimize_and_publish(self):
        from gtsam import LevenbergMarquardtOptimizer
        optimizer = LevenbergMarquardtOptimizer(self.graph, self.initial_estimates)
        result = optimizer.optimize()

        latest_pose = result.atPose2(self.pose_id - 1)
        pose_msg = PoseStamped()
        pose_msg.header = Header(frame_id='ned', stamp=self.get_clock().now().to_msg())
        pose_msg.pose.position.x = latest_pose.x()  # North
        pose_msg.pose.position.y = latest_pose.y()  # East
        pose_msg.pose.orientation = self.yaw_to_quat(latest_pose.theta())
        self.pose_pub.publish(pose_msg)

        self.initial_estimates = result
        self.pose_id += 1

    def quat_to_yaw(self, quat):
        import tf_transformations
        return tf_transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])[2]

    def yaw_to_quat(self, yaw):
        import tf_transformations
        q = tf_transformations.quaternion_from_euler(0, 0, yaw)
        from geometry_msgs.msg import Quaternion
        return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

def main(args=None):
    rclpy.init(args=args)
    fusion_node = GICPGPSFusionNED()
    rclpy.spin(fusion_node)
    fusion_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()