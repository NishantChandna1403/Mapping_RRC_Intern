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
#pragma once

// GenZ-ICP
#include "genz_icp/pipeline/GenZICP.hpp"

// ROS 2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>  // Added for GPS local position

#include <memory>
#include <string>
#include <vector>

namespace genz_icp_ros {

class OdometryServer : public rclcpp::Node {
public:
    /// OdometryServer constructor
    OdometryServer() = delete;
    explicit OdometryServer(const rclcpp::NodeOptions& options);

private:
    /// Register new LiDAR frame
    void RegisterFrame(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg);

    /// Callback for processing GPS local position
    void GpsLocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);

    /// Stream the estimated pose to ROS
    void PublishOdometry(const Sophus::SE3d& pose,
                         const rclcpp::Time& stamp,
                         const std::string& cloud_frame_id);

    /// Stream the debugging point clouds for visualization (if required)
    void PublishClouds(const rclcpp::Time& stamp,
                       const std::string& cloud_frame_id,
                       const std::vector<Eigen::Vector3d>& planar_points,
                       const std::vector<Eigen::Vector3d>& non_planar_points);

    /// Utility function to compute transformation using tf tree
    Sophus::SE3d LookupTransform(const std::string& target_frame,
                                 const std::string& source_frame) const;

private:
    /// Tools for broadcasting TFs
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;           // Changed to shared_ptr to match CPP
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;  // Changed to shared_ptr to match CPP

    /// Parameters
    std::string base_frame_{""};
    std::string odom_frame_{"odom"};
    bool publish_odom_tf_{true};
    bool publish_debug_clouds_{false};
    
    // Configuration parameters for GenZICP
    double max_range_{100.0};
    double min_range_{0.0};
    bool deskew_{false};
    double voxel_size_{1.0};
    double map_cleanup_radius_{50.0};
    double planarity_threshold_{0.1};
    int max_points_per_voxel_{20};
    int desired_num_voxelized_points_{10000};
    int max_num_iterations_{30};
    double convergence_criterion_{0.001};
    double initial_threshold_{2.0};
    double min_motion_th_{0.01};

    /// GPS fusion variables
    bool initial_position_set_{false};
    Eigen::Vector3d initial_ned_position_{Eigen::Vector3d::Zero()};
    px4_msgs::msg::VehicleLocalPosition current_gps_local_pos_;

    /// Data subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr gps_local_pos_sub_;

    /// Data publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr traj_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr planar_points_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr non_planar_points_publisher_;

    /// Path message
    nav_msgs::msg::Path path_msg_;

    /// GenZ-ICP odometry
    genz_icp::pipeline::GenZICP odometry_;
};

}  // namespace genz_icp_ros