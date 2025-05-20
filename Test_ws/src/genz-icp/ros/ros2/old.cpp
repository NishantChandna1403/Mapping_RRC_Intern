#include <Eigen/Core>
#include <memory>
#include <sophus/se3.hpp>
#include <utility>
#include <vector>

// GenZ-ICP-ROS
#include "OdometryServer.hpp"
#include "Utils.hpp"

// GenZ-ICP
#include "genz_icp/pipeline/GenZICP.hpp"

// ROS 2 headers
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>

namespace genz_icp_ros {

using utils::EigenToPointCloud2;
using utils::GetTimestamps;
using utils::PointCloud2ToEigen;

OdometryServer::OdometryServer(const rclcpp::NodeOptions &options)
    : rclcpp::Node("odometry_node", options) {
    // clang-format off
    base_frame_ = declare_parameter<std::string>("base_frame", base_frame_);
    odom_frame_ = declare_parameter<std::string>("odom_frame", odom_frame_);
    publish_odom_tf_ = declare_parameter<bool>("publish_odom_tf", publish_odom_tf_);
    publish_debug_clouds_ = declare_parameter<bool>("visualize", publish_debug_clouds_);
    config_.max_range = declare_parameter<double>("max_range", config_.max_range);
    config_.min_range = declare_parameter<double>("min_range", config_.min_range);
    config_.deskew = declare_parameter<bool>("deskew", config_.deskew);
    config_.voxel_size = declare_parameter<double>("voxel_size", config_.max_range / 100.0);
    config_.map_cleanup_radius = declare_parameter<double>("map_cleanup_radius", config_.map_cleanup_radius);
    config_.planarity_threshold = declare_parameter<double>("planarity_threshold", config_.planarity_threshold);
    config_.max_points_per_voxel = declare_parameter<int>("max_points_per_voxel", config_.max_points_per_voxel);
    config_.desired_num_voxelized_points = declare_parameter<int>("desired_num_voxelized_points", config_.desired_num_voxelized_points);
    config_.max_num_iterations = declare_parameter<int>("max_num_iterations", config_.max_num_iterations);
    config_.convergence_criterion = declare_parameter<double>("convergence_criterion", config_.convergence_criterion);
    config_.initial_threshold = declare_parameter<double>("initial_threshold", config_.initial_threshold);
    config_.min_motion_th = declare_parameter<double>("min_motion_th", config_.min_motion_th);
    if (config_.max_range < config_.min_range) {
        RCLCPP_WARN(get_logger(), "[WARNING] max_range is smaller than min_range, settng min_range to 0.0");
        config_.min_range = 0.0;
    }
    // clang-format on

    // Construct the main GenZ-ICP odometry node
    odometry_ = genz_icp::pipeline::GenZICP(config_);

    // Initialize subscribers
    pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "pointcloud_topic", rclcpp::SensorDataQoS(),
        std::bind(&OdometryServer::RegisterFrame, this, std::placeholders::_1));

    attitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
        "/px4_1/fmu/out/vehicle_attitude", rclcpp::SensorDataQoS(),
        std::bind(&OdometryServer::AttitudeCallback, this, std::placeholders::_1));

    local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/px4_1/fmu/out/vehicle_local_position", rclcpp::SensorDataQoS(),
        std::bind(&OdometryServer::LocalPositionCallback, this, std::placeholders::_1));

    sensor_combined_sub_ = create_subscription<px4_msgs::msg::SensorCombined>(
        "/px4_1/fmu/out/sensor_combined", rclcpp::SensorDataQoS(),
        std::bind(&OdometryServer::SensorCombinedCallback, this, std::placeholders::_1));

    // Initialize publishers
    rclcpp::QoS qos((rclcpp::SystemDefaultsQoS().keep_last(1).durability_volatile()));
    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("/genz/odometry", qos);
    traj_publisher_ = create_publisher<nav_msgs::msg::Path>("/genz/trajectory", qos);
    attitude_odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("/attitude/odometry", qos);
    path_msg_.header.frame_id = odom_frame_;
    if (publish_debug_clouds_) {
        map_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("/genz/local_map", qos);
        planar_points_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("/genz/planar_points", qos);
        non_planar_points_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("/genz/non_planar_points", qos);
    }

    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf2_buffer_->setUsingDedicatedThread(true);
    tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_buffer_);

    RCLCPP_INFO(this->get_logger(), "GenZ-ICP ROS 2 odometry node initialized");
}

Sophus::SE3d OdometryServer::LookupTransform(const std::string &target_frame,
                                             const std::string &source_frame) const {
    std::string err_msg;
    if (tf2_buffer_->_frameExists(source_frame) &&  //
        tf2_buffer_->_frameExists(target_frame) &&  //
        tf2_buffer_->canTransform(target_frame, source_frame, tf2::TimePointZero, &err_msg)) {
        try {
            auto tf = tf2_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
            return tf2::transformToSophus(tf);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "%s", ex.what());
        }
    }
    RCLCPP_WARN(this->get_logger(), "Failed to find tf. Reason=%s", err_msg.c_str());
    return {};
}

void OdometryServer::AttitudeCallback(const px4_msgs::msg::VehicleAttitude::ConstSharedPtr &msg) {
    nav_msgs::msg::Odometry attitude_odom_msg;
    attitude_odom_msg.header.stamp = this->now();
    attitude_odom_msg.header.frame_id = odom_frame_;
    attitude_odom_msg.child_frame_id = base_frame_.empty() ? "attitude_link" : base_frame_;

    attitude_odom_msg.pose.pose.position.x = 0.0;
    attitude_odom_msg.pose.pose.position.y = 0.0;
    attitude_odom_msg.pose.pose.position.z = 0.0;

    Eigen::Quaterniond ned_quat(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
    Eigen::Quaterniond ned_to_enu = 
        Eigen::Quaterniond(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ())) *
        Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
    Eigen::Quaterniond enu_quat = ned_to_enu * ned_quat * ned_to_enu.inverse();
    enu_quat.normalize();

    attitude_odom_msg.pose.pose.orientation.w = enu_quat.w();
    attitude_odom_msg.pose.pose.orientation.x = enu_quat.x();
    attitude_odom_msg.pose.pose.orientation.y = enu_quat.y();
    attitude_odom_msg.pose.pose.orientation.z = enu_quat.z();

    Eigen::Vector3d translation(0.0, 0.0, 0.0);
    latest_attitude_pose_ = Sophus::SE3d(enu_quat, translation);

    if (!first_attitude_received_) {
        first_attitude_received_ = true;
    }

    attitude_odom_publisher_->publish(attitude_odom_msg);
}

void OdometryServer::LocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition::ConstSharedPtr &msg) {
    Eigen::Vector3d ned_translation(msg->x, msg->y, msg->z);
    Eigen::Vector3d enu_translation;
    enu_translation.x() = ned_translation.y();
    enu_translation.y() = ned_translation.x();
    enu_translation.z() = -ned_translation.z();

    Eigen::Quaterniond quaternion;
    if (first_attitude_received_) {
        quaternion = latest_attitude_pose_.unit_quaternion();
    } else {
        quaternion = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
    }
    
    latest_local_position_pose_ = Sophus::SE3d(quaternion, enu_translation);

    if (!first_local_position_received_) {
        first_local_position_received_ = true;
    }
}

void OdometryServer::SensorCombinedCallback(const px4_msgs::msg::SensorCombined::ConstSharedPtr &msg) {
    Eigen::Vector3d ned_gyro(msg->gyro_rad[0], msg->gyro_rad[1], msg->gyro_rad[2]);
    Eigen::Vector3d ned_accel(msg->accelerometer_m_s2[0], msg->accelerometer_m_s2[1], msg->accelerometer_m_s2[2]);

    Eigen::Matrix3d ned_to_enu_rot = 
        Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
        Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
    
    latest_imu_data_.gyro = ned_to_enu_rot * ned_gyro;
    latest_imu_data_.accel = ned_to_enu_rot * ned_accel;
    latest_imu_data_.timestamp = msg->timestamp / 1e6;

    if (!first_sensor_combined_received_) {
        first_sensor_combined_received_ = true;
    }
}

void OdometryServer::RegisterFrame(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
    const auto cloud_frame_id = msg->header.frame_id;
    const auto points = PointCloud2ToEigen(msg);
    const auto timestamps = [&]() -> std::vector<double> {
        if (!config_.deskew) return {};
        return GetTimestamps(msg);
    }();
    const auto egocentric_estimation = (base_frame_.empty() || base_frame_ == cloud_frame_id);

    double cloud_timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec / 1e9;
    RCLCPP_INFO(this->get_logger(), "Timestamp diff (cloud - IMU): %f", cloud_timestamp - latest_imu_data_.timestamp);

    RCLCPP_INFO(this->get_logger(), 
                "IMU - Gyro: [%f, %f, %f], Accel: [%f, %f, %f], Timestamp: %f",
                latest_imu_data_.gyro.x(),
                latest_imu_data_.gyro.y(),
                latest_imu_data_.gyro.z(),
                latest_imu_data_.accel.x(),
                latest_imu_data_.accel.y(),
                latest_imu_data_.accel.z(),
                latest_imu_data_.timestamp);

    // Use the 8-argument overload of RegisterFrame
    double eph = config_.gps_accuracy; // Default GPS horizontal accuracy
    double epv = config_.gps_accuracy; // Default GPS vertical accuracy
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero(); // Default velocity
    double dt = first_sensor_combined_received_ ? (cloud_timestamp - latest_imu_data_.timestamp) : 0.0;

    const auto &[planar_points, non_planar_points] = odometry_.RegisterFrame(
        points, timestamps,
        first_attitude_received_ ? latest_attitude_pose_ : Sophus::SE3d(),
        first_local_position_received_ ? latest_local_position_pose_ : Sophus::SE3d(),
        eph, epv, velocity, dt);

    const Sophus::SE3d genz_pose = odometry_.poses().back();

    const auto pose = [&]() -> Sophus::SE3d {
        if (egocentric_estimation) return genz_pose;
        const Sophus::SE3d cloud2base = LookupTransform(base_frame_, cloud_frame_id);
        return cloud2base * genz_pose * cloud2base.inverse();
    }();

    PublishOdometry(pose, msg->header.stamp, cloud_frame_id);
    if (publish_debug_clouds_) {
        PublishClouds(msg->header.stamp, cloud_frame_id, planar_points, non_planar_points);
    }
}

void OdometryServer::PublishOdometry(const Sophus::SE3d &pose,
                                     const rclcpp::Time &stamp,
                                     const std::string &cloud_frame_id) {
    if (publish_odom_tf_) {
        geometry_msgs::msg::TransformStamped transform_msg;
        transform_msg.header.stamp = stamp;
        transform_msg.header.frame_id = odom_frame_;
        transform_msg.child_frame_id = base_frame_.empty() ? cloud_frame_id : base_frame_;
        transform_msg.transform = tf2::sophusToTransform(pose);
        tf_broadcaster_->sendTransform(transform_msg);
    }

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = odom_frame_;
    pose_msg.pose = tf2::sophusToPose(pose);
    path_msg_.poses.push_back(pose_msg);
    traj_publisher_->publish(path_msg_);

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.pose.pose = tf2::sophusToPose(pose);
    odom_publisher_->publish(std::move(odom_msg));
}

void OdometryServer::PublishClouds(const rclcpp::Time &stamp,
                                   const std::string &cloud_frame_id,
                                   const std::vector<Eigen::Vector3d> &planar_points,
                                   const std::vector<Eigen::Vector3d> &non_planar_points) {
    std_msgs::msg::Header odom_header;
    odom_header.stamp = stamp;
    odom_header.frame_id = odom_frame_;

    const auto genz_map = odometry_.LocalMap();

    if (!publish_odom_tf_) {
        std_msgs::msg::Header cloud_header;
        cloud_header.stamp = stamp;
        cloud_header.frame_id = cloud_frame_id;

        map_publisher_->publish(std::move(EigenToPointCloud2(genz_map, odom_header)));
        planar_points_publisher_->publish(std::move(EigenToPointCloud2(planar_points, cloud_header)));
        non_planar_points_publisher_->publish(std::move(EigenToPointCloud2(non_planar_points, cloud_header)));
        return;
    }

    const auto cloud2odom = LookupTransform(odom_frame_, cloud_frame_id);
    planar_points_publisher_->publish(std::move(EigenToPointCloud2(planar_points, odom_header)));
    non_planar_points_publisher_->publish(std::move(EigenToPointCloud2(non_planar_points, odom_header)));

    if (!base_frame_.empty()) {
        const Sophus::SE3d cloud2base = LookupTransform(base_frame_, cloud_frame_id);
        map_publisher_->publish(std::move(EigenToPointCloud2(genz_map, cloud2base, odom_header)));
    } else {
        map_publisher_->publish(std::move(EigenToPointCloud2(genz_map, odom_header)));
    }
}

}  // namespace genz_icp_ros

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(genz_icp_ros::OdometryServer)