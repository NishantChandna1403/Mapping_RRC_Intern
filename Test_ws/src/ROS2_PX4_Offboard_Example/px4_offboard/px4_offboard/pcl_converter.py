import rclpy
from rclpy.node import Node
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header

class PointCloudTransformer(Node):
    def __init__(self):
        super().__init__('pointcloud_transformer')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera/points',
            self.pointcloud_callback,
            50
        )
        self.publisher = self.create_publisher(PointCloud2, '/camera/points_transformed', 50)
        
        # Transformation matrix: Camera Optical (Z forward, X right, Y down) -> ROS Standard (X forward, Y left, Z up)
        self.transformation_matrix = np.array([
            [0, 0, 1, 0],  # Camera Z -> ROS X
            [-1, 0, 0, 0], # Camera X -> -ROS Y
            [0, -1, 0, 0], # Camera Y -> -ROS Z
            [0, 0, 0, 1]   # Homogeneous transformation
        ], dtype=np.float32)
        
        self.get_logger().info('PointCloud transformer node initialized')

    def pointcloud_callback(self, msg):
        self.get_logger().debug(f'Received point cloud with {msg.width * msg.height} points')

        # Convert PointCloud2 to a structured NumPy array
        cloud_data = np.fromiter(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True),
                                 dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32)])
        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = "camera_link"

        if cloud_data.shape[0] == 0:
            self.get_logger().warn('Received empty point cloud, publishing empty transformed cloud')
            new_msg = pc2.create_cloud_xyz32(header, [])  # Empty cloud
            self.publisher.publish(new_msg)
            return
        cloud_points = np.column_stack((cloud_data['x'], cloud_data['y'], cloud_data['z']))
        valid_mask = cloud_points[:, 2] <= 50.0
        filtered_points = cloud_points[valid_mask]
        valid_mask = filtered_points[:, 1] <= -1 
        filtered_points = filtered_points[valid_mask]
        
        if filtered_points.shape[0] == 0:
            self.get_logger().warn('All points filtered out (z > 50m or y > -1), publishing empty transformed cloud')
            new_msg = pc2.create_cloud_xyz32(header, [])  # Empty cloud
            self.publisher.publish(new_msg)
            return
        ones = np.ones((filtered_points.shape[0], 1), dtype=np.float32)
        cloud_points_hom = np.hstack((filtered_points, ones))  # Shape (N, 4)
        transformed_points = (self.transformation_matrix @ cloud_points_hom.T).T[:, :3]  # (N, 3)
        
        # Create and publish new PointCloud2 message
        new_msg = pc2.create_cloud_xyz32(header, transformed_points.tolist())
        self.publisher.publish(new_msg)
        self.get_logger().debug(f'Published transformed point cloud with {transformed_points.shape[0]} points')

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudTransformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()