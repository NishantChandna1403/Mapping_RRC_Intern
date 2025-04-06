// MIT License
//
// Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill Stachniss.
// Modified by Daehan Lee, Hyungtae Lim, and Soohee Han, 2024
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
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
#include <px4_msgs/msg/vehicle_local_position.hpp> // Added for GPS local position

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
    max_range_ = declare_parameter<double>("max_range", max_range_);
    min_range_ = declare_parameter<double>("min_range", min_range_);
    deskew_ = declare_parameter<bool>("deskew", deskew_);
    voxel_size_ = declare_parameter<double>("voxel_size", max_range_ / 100.0);
    map_cleanup_radius_ = declare_parameter<double>("map_cleanup_radius", map_cleanup_radius_);
    planarity_threshold_ = declare_parameter<double>("planarity_threshold", planarity_threshold_);
    max_points_per_voxel_ = declare_parameter<int>("max_points_per_voxel", max_points_per_voxel_);
    desired_num_voxelized_points_ = declare_parameter<int>("desired_num_voxelized_points", desired_num_voxelized_points_);
    max_num_iterations_ = declare_parameter<int>("max_num_iterations", max_num_iterations_);
    convergence_criterion_ = declare_parameter<double>("convergence_criterion", convergence_criterion_);
    initial_threshold_ = declare_parameter<double>("initial_threshold", initial_threshold_);
    min_motion_th_ = declare_parameter<double>("min_motion_th", min_motion_th_);
    if (max_range_ < min_range_) {
        RCLCPP_WARN(get_logger(), "[WARNING] max_range is smaller than min_range, settng min_range to 0.0");
        min_range_ = 0.0;
    }
    // clang-format on

    // Create a configuration for GenZICP
    genz_icp::pipeline::GenZConfig config;  // Use non-nested Config
    config.max_range = max_range_;
    config.min_range = min_range_;
    config.deskew = deskew_;
    config.voxel_size = voxel_size_;
    config.map_cleanup_radius = map_cleanup_radius_;
    config.planarity_threshold = planarity_threshold_;
    config.max_points_per_voxel = max_points_per_voxel_;
    config.desired_num_voxelized_points = desired_num_voxelized_points_;
    config.max_num_iterations = max_num_iterations_;
    config.convergence_criterion = convergence_criterion_;
    config.initial_threshold = initial_threshold_;
    config.min_motion_th = min_motion_th_;

    // Construct the main GenZ-ICP odometry node
    odometry_ = genz_icp::pipeline::GenZICP(config);
    
    // Initialize flag to track if initial position is set
    initial_position_set_ = false;

    // Initialize subscribers
    pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "pointcloud_topic", rclcpp::SensorDataQoS(),
        std::bind(&OdometryServer::RegisterFrame, this, std::placeholders::_1));
    
    // Subscribe to GPS local position
    gps_local_pos_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position", rclcpp::SensorDataQoS(),
        std::bind(&OdometryServer::GpsLocalPositionCallback, this, std::placeholders::_1));

    // Initialize publishers
    rclcpp::QoS qos((rclcpp::SystemDefaultsQoS().keep_last(1).durability_volatile()));
    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("/genz/odometry", qos);
    traj_publisher_ = create_publisher<nav_msgs::msg::Path>("/genz/trajectory", qos);
    path_msg_.header.frame_id = odom_frame_;
    if (publish_debug_clouds_) {
        map_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("/genz/local_map", qos);
        planar_points_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("/genz/planar_points", qos);
        non_planar_points_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("/genz/non_planar_points", qos);
    }

    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf2_buffer_->setUsingDedicatedThread(true);
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

    RCLCPP_INFO(this->get_logger(), "GenZ-ICP ROS 2 odometry node initialized with GPS fusion");
}

void OdometryServer::GpsLocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
    // Store the latest GPS local position message
    current_gps_local_pos_ = *msg;
    
    // Set the initial position frame as reference frame if not set yet
    if (!initial_position_set_ && msg->xy_valid && msg->z_valid) {
        // Store initial NED position as reference frame
        initial_ned_position_ = Eigen::Vector3d(msg->x, msg->y, msg->z);
        initial_position_set_ = true;
        RCLCPP_INFO(this->get_logger(), "Initial NED position set as reference frame: [%f, %f, %f]", 
                    initial_ned_position_[0], initial_ned_position_[1], initial_ned_position_[2]);
    }
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

void OdometryServer::RegisterFrame(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
    const auto cloud_frame_id = msg->header.frame_id;
    const auto points = PointCloud2ToEigen(msg);
    const auto timestamps = [&]() -> std::vector<double> {
        if (!deskew_) return {};
        return GetTimestamps(msg);
    }();
    const auto egocentric_estimation = (base_frame_.empty() || base_frame_ == cloud_frame_id);

    // Check if we have enough points for reliable ICP
    bool use_gps_only = points.size() < 400;
    
    // If we don't have enough points or haven't set initial position yet, 
    // just use GPS for odometry if available
    if (use_gps_only || !initial_position_set_) {
        if (current_gps_local_pos_.xy_valid && current_gps_local_pos_.z_valid) {
            // Create a pose from GPS local position
            Eigen::Vector3d gps_position(
                current_gps_local_pos_.x - initial_ned_position_[0],
                current_gps_local_pos_.y - initial_ned_position_[1],
                current_gps_local_pos_.z - initial_ned_position_[2]
            );
            
            // Create rotation matrix from GPS heading
            double heading = current_gps_local_pos_.heading;
            Eigen::Quaterniond q = Eigen::Quaterniond(
                Eigen::AngleAxisd(heading, Eigen::Vector3d::UnitZ())
            );
            
            Sophus::SE3d gps_pose(q, gps_position);
            
            // Publish the GPS-based odometry
            PublishOdometry(gps_pose, msg->header.stamp, cloud_frame_id);
            
            RCLCPP_INFO(this->get_logger(), "Using GPS-only odometry (points: %zu)", points.size());
            return;
        } else {
            RCLCPP_WARN(this->get_logger(), "No valid GPS position and not enough points for ICP");
            return;
        }
    }

    // Register frame, main entry point to GenZ-ICP pipeline
    const auto &[planar_points, non_planar_points] = odometry_.RegisterFrame(points, timestamps);

    // Compute the pose using GenZ, ego-centric to the LiDAR
    const Sophus::SE3d icp_pose = odometry_.poses().back();

    // If necessary, transform the ego-centric pose to the specified base_link/base_footprint frame
    const auto icp_pose_transformed = [&]() -> Sophus::SE3d {
        if (egocentric_estimation) return icp_pose;
        const Sophus::SE3d cloud2base = LookupTransform(base_frame_, cloud_frame_id);
        return cloud2base * icp_pose * cloud2base.inverse();
    }();

    // Create GPS pose if available
    Sophus::SE3d final_pose;
    if (current_gps_local_pos_.xy_valid && current_gps_local_pos_.z_valid) {
        // Create a pose from GPS local position
        Eigen::Vector3d gps_position(
            current_gps_local_pos_.x - initial_ned_position_[0],
            current_gps_local_pos_.y - initial_ned_position_[1],
            current_gps_local_pos_.z - initial_ned_position_[2]
        );
        
        // Create rotation matrix from GPS heading
        double heading = current_gps_local_pos_.heading;
        Eigen::Quaterniond q = Eigen::Quaterniond(
            Eigen::AngleAxisd(heading, Eigen::Vector3d::UnitZ())
        );
        
        Sophus::SE3d gps_pose(q, gps_position);
        
        // Fuse GPS and ICP poses with equal weights (0.5 each)
        // For translation, we can simply interpolate
        Eigen::Vector3d fused_translation = 0.5 * icp_pose_transformed.translation() + 0.5 * gps_pose.translation();
        
        // For rotation, we can use SLERP for quaternions
        Eigen::Quaterniond icp_q = icp_pose_transformed.unit_quaternion();
        Eigen::Quaterniond gps_q = gps_pose.unit_quaternion();
        Eigen::Quaterniond fused_q = icp_q.slerp(0.5, gps_q);
        
        final_pose = Sophus::SE3d(fused_q, fused_translation);
        
        RCLCPP_INFO(this->get_logger(), "Fusing GPS and ICP odometry (50/50)");
    } else {
        // If no valid GPS, use ICP pose only
        final_pose = icp_pose_transformed;
        RCLCPP_INFO(this->get_logger(), "Using ICP-only odometry (no valid GPS)");
    }

    // Spit the current estimated pose to ROS msgs
    PublishOdometry(final_pose, msg->header.stamp, cloud_frame_id);
    
    // Publishing this clouds is a bit costly, so do it only if we are debugging
    if (publish_debug_clouds_) {
        PublishClouds(msg->header.stamp, cloud_frame_id, planar_points, non_planar_points);
    }
}

void OdometryServer::PublishOdometry(const Sophus::SE3d &pose,
                                     const rclcpp::Time &stamp,
                                     const std::string &cloud_frame_id) {
    // Broadcast the tf ---
    if (publish_odom_tf_) {
        geometry_msgs::msg::TransformStamped transform_msg;
        transform_msg.header.stamp = stamp;
        transform_msg.header.frame_id = odom_frame_;
        transform_msg.child_frame_id = base_frame_.empty() ? cloud_frame_id : base_frame_;
        transform_msg.transform = tf2::sophusToTransform(pose);
        tf_broadcaster_->sendTransform(transform_msg);
    }

    // publish trajectory msg
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = odom_frame_;
    pose_msg.pose = tf2::sophusToPose(pose);
    path_msg_.poses.push_back(pose_msg);
    traj_publisher_->publish(path_msg_);

    // publish odometry msg
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

    // Publish map
    const auto genz_map = odometry_.LocalMap();

    if (!publish_odom_tf_) {
        // debugging happens in an egocentric world
        std_msgs::msg::Header cloud_header;
        cloud_header.stamp = stamp;
        cloud_header.frame_id = cloud_frame_id;

        map_publisher_->publish(std::move(EigenToPointCloud2(genz_map, odom_header)));
        planar_points_publisher_->publish(std::move(EigenToPointCloud2(planar_points, cloud_header)));
        non_planar_points_publisher_->publish(std::move(EigenToPointCloud2(non_planar_points, cloud_header)));

        return;
    }

    // If transmitting to tf tree we know where the clouds are exactly
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