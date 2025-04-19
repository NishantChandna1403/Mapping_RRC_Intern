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

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

#include <Eigen/Core>
#include <memory>
#include <string>

#include "genz_icp/pipeline/GenZICP.hpp"

namespace genz_icp_ros {

class OdometryServer : public rclcpp::Node {
public:
    explicit OdometryServer(const rclcpp::NodeOptions &options);

    Sophus::SE3d LookupTransform(const std::string &target_frame, const std::string &source_frame) const;

private:
    void RegisterFrame(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
    void AttitudeCallback(const px4_msgs::msg::VehicleAttitude::ConstSharedPtr &msg);
    void LocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition::ConstSharedPtr &msg);
    void PublishOdometry(const Sophus::SE3d &pose, const rclcpp::Time &stamp, const std::string &cloud_frame_id);
    void PublishClouds(const rclcpp::Time &stamp,
                       const std::string &cloud_frame_id,
                       const std::vector<Eigen::Vector3d> &planar_points,
                       const std::vector<Eigen::Vector3d> &non_planar_points);

private:
    genz_icp::pipeline::GenZICP odometry_;
    genz_icp::pipeline::GenZConfig config_;
    std::string base_frame_ = "base_link";
    std::string odom_frame_ = "odom";
    bool publish_odom_tf_ = true;
    bool publish_debug_clouds_ = false;

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;

    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr traj_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr imu_odom_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr planar_points_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr non_planar_points_publisher_;

    // TF
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;

    // State
    nav_msgs::msg::Path path_msg_;
    bool first_imu_received_ = false;
    bool first_local_position_received_ = false;
    Sophus::SE3d latest_imu_pose_;
    Sophus::SE3d latest_local_position_pose_;
    double latest_imu_time_ = 0.0;
    Eigen::Vector3d latest_velocity_ = Eigen::Vector3d::Zero();
    double latest_eph_ = 1.0;
    double latest_epv_ = 1.0;
    double latest_gps_time_ = 0.0;
};

} 