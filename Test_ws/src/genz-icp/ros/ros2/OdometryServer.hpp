#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>
#include <deque>
#include <memory>
#include <string>

#include "genz_icp/pipeline/GenZICP.hpp"
#include "genz_icp/pipeline/Preintegration.hpp"

namespace genz_icp_ros {

struct IMUData {
    Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel = Eigen::Vector3d::Zero();
    double timestamp = 0.0;
};

class OdometryServer : public rclcpp::Node {
public:
    explicit OdometryServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
    Sophus::SE3d LookupTransform(const std::string &target_frame, const std::string &source_frame) const;

    void AttitudeCallback(const px4_msgs::msg::VehicleAttitude::ConstSharedPtr &msg);
    void LocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition::ConstSharedPtr &msg);
    void SensorCombinedCallback(const px4_msgs::msg::SensorCombined::ConstSharedPtr &msg);
    void RegisterFrame(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);

    void PublishOdometry(const Sophus::SE3d &pose, const rclcpp::Time &stamp, const std::string &cloud_frame_id);
    void PublishClouds(const rclcpp::Time &stamp, const std::string &cloud_frame_id,
                       const std::vector<Eigen::Vector3d> &planar_points,
                       const std::vector<Eigen::Vector3d> &non_planar_points);

private:
    genz_icp::pipeline::GenZConfig config_;
    genz_icp::pipeline::GenZICP odometry_;
    std::unique_ptr<genz_icp::pipeline::Preintegration> preintegration_;

    std::string base_frame_ = "base_link";
    std::string odom_frame_ = "odom";
    bool publish_odom_tf_ = true;
    bool publish_debug_clouds_ = true;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
    rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr sensor_combined_sub_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr traj_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr attitude_odom_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr planar_points_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr non_planar_points_publisher_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;

    nav_msgs::msg::Path path_msg_;
    Sophus::SE3d latest_attitude_pose_;
    Sophus::SE3d latest_local_position_pose_;
    Eigen::Vector3d latest_velocity_;
    std::deque<IMUData> imu_buffer_;
    double last_keyframe_timestamp_ = 0.0;

    bool first_attitude_received_ = false;
    bool first_local_position_received_ = false;
    bool first_sensor_combined_received_ = false;
};

} // namespace genz_icp_ros