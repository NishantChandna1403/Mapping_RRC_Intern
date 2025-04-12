// SPDX-License-Identifier: MIT
// Copyright (c) 2024 Ignacio Vizzo, Cyrill Stachniss, University of Bonn
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

#include "GenZICP.hpp"

#include <Eigen/Core>
#include <tuple>
#include <vector>
#include <iostream>

#include "genz_icp/core/Deskew.hpp"
#include "genz_icp/core/Preprocessing.hpp"
#include "genz_icp/core/Registration.hpp"
#include "genz_icp/core/VoxelHashMap.hpp"

namespace genz_icp::pipeline {

Sophus::SE3d GenZICP::fusePoses(const Sophus::SE3d& icp_pose,
                                const Sophus::SE3d& imu_orientation,
                                const Sophus::SE3d& local_position_pose,
                                size_t num_points) {
    std::cout << "[DEBUG] Fusing poses with " << num_points << " points" << std::endl;

    // Compute ICP weightage based on number of points
    const double threshold_points = 200000.0;
    double icp_weight;
    if (static_cast<double>(num_points) >= threshold_points) {
        icp_weight = 1.0;

    
    } else {
        icp_weight = static_cast<double>(num_points) / threshold_points;
        icp_weight = std::min(0.99, icp_weight); // Cap at 0.99 to ensure some GPS+IMU contribution
    }
    double gps_imu_weight = 1.0 - icp_weight;

    std::cout << "[DEBUG] Weights - ICP: " << icp_weight << ", GPS+IMU: " << gps_imu_weight << std::endl;

    // GPS+IMU pose: GPS translation + IMU orientation
    Sophus::SE3d gps_imu_pose(imu_orientation.unit_quaternion(), local_position_pose.translation());

    if (icp_weight >= 1.0) {
        // Use ICP pose exclusively
        std::cout << "[DEBUG] Using full ICP pose - Translation: " << icp_pose.translation().transpose()
                  << ", Quaternion: " << icp_pose.unit_quaternion().coeffs().transpose() << std::endl;
        return icp_pose;
    }

    // Fuse translation (weighted average)
    Eigen::Vector3d fused_trans = icp_weight * icp_pose.translation() +
                                  gps_imu_weight * gps_imu_pose.translation();

    // Fuse orientation (slerp between quaternions)
    Eigen::Quaterniond icp_quat = icp_pose.unit_quaternion();
    Eigen::Quaterniond gps_imu_quat = gps_imu_pose.unit_quaternion();
    Eigen::Quaterniond fused_quat = icp_quat.slerp(gps_imu_weight, gps_imu_quat);
    fused_quat.normalize();

    std::cout << "[DEBUG] Fused - Translation: " << fused_trans.transpose()
              << ", Quaternion: " << fused_quat.coeffs().transpose() << std::endl;

    return Sophus::SE3d(fused_quat, fused_trans);
}

GenZICP::Vector3dVectorTuple GenZICP::RegisterFrame(const std::vector<Eigen::Vector3d> &frame,
                                                   const std::vector<double> timestamps,
                                                   const Sophus::SE3d &imu_orientation,
                                                   const Sophus::SE3d &local_position_pose) {
    std::cout << "[DEBUG] Entering RegisterFrame" << std::endl;
    std::cout << "[DEBUG] Raw input frame size: " << frame.size() << std::endl;

    const auto &deskew_frame = [&]() -> std::vector<Eigen::Vector3d> {
        if (!config_.deskew || timestamps.empty()) return frame;
        const size_t N = poses().size();
        if (N <= 2) return frame;
        const auto &start_pose = poses_[N - 2];
        const auto &finish_pose = poses_[N - 1];
        std::cout << "[DEBUG] Deskewing frame" << std::endl;
        return DeSkewScan(frame, timestamps, start_pose, finish_pose);
    }();

    std::cout << "[DEBUG] Frame points after deskew: " << deskew_frame.size() << std::endl;

    Sophus::SE3d new_pose;
    std::vector<Eigen::Vector3d> planar_points, non_planar_points;

    // Always attempt ICP, but weightage will determine how much it's used
    std::cout << "[DEBUG] Entering ICP + GPS + IMU mode" << std::endl;
    Sophus::SE3d icp_pose;
    try {
        const auto &cropped_frame = Preprocess(deskew_frame, config_.max_range, config_.min_range);
        std::cout << "[DEBUG] Cropped points: " << cropped_frame.size() << std::endl;

        static double voxel_size = config_.voxel_size;
        const auto source_tmp = genz_icp::VoxelDownsample(cropped_frame, voxel_size);
        double adaptive_voxel_size = genz_icp::Clamp(
            voxel_size * static_cast<double>(source_tmp.size()) / static_cast<double>(config_.desired_num_voxelized_points), 0.02, 2.0);
        const auto &[source, frame_downsample] = Voxelize(cropped_frame, adaptive_voxel_size);
        voxel_size = adaptive_voxel_size;
        std::cout << "[DEBUG] Voxelized source points: " << source.size() << std::endl;

        const double sigma = GetAdaptiveThreshold();

        Sophus::SE3d initial_guess;
        if (first_frame_) {
            initial_enu_orientation_ = imu_orientation;
            initial_guess = initial_enu_orientation_;
            last_imu_orientation_ = imu_orientation;
            first_frame_ = false;
            std::cout << "[DEBUG] First frame initialized in ICP" << std::endl;
        } else {
            const auto prediction = GetPredictionModel();
            const auto last_pose = poses_.back();
            bool is_imu_identity = imu_orientation.unit_quaternion().isApprox(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0), 1e-6) &&
                                  imu_orientation.translation().norm() < 1e-6;
            Sophus::SE3d imu_delta = last_imu_orientation_.inverse() * imu_orientation;
            initial_guess = is_imu_identity ? last_pose * prediction : last_pose * imu_delta;
            last_imu_orientation_ = imu_orientation;
        }
        std::cout << "[DEBUG] Initial guess computed" << std::endl;

        std::cout << "[DEBUG] Running ICP registration" << std::endl;
        auto registration_result = registration_.RegisterFrame(source, local_map_, initial_guess, 3.0 * sigma, sigma / 3.0);
        std::tie(icp_pose, planar_points, non_planar_points) = registration_result;
        std::cout << "[DEBUG] ICP pose - Translation: " << icp_pose.translation().transpose()
                  << ", Quaternion: " << icp_pose.unit_quaternion().coeffs().transpose() << std::endl;

        // Fuse poses using point-based weightage
        new_pose = fusePoses(icp_pose, imu_orientation, local_position_pose, deskew_frame.size());

        std::cout << "[DEBUG] Updating local map and threshold" << std::endl;
        const auto model_deviation = initial_guess.inverse() * new_pose;
        adaptive_threshold_.UpdateModelDeviation(model_deviation);
        local_map_.Update(frame_downsample, new_pose);
    } catch (const std::exception& e) {
        new_pose = Sophus::SE3d(imu_orientation.unit_quaternion(), local_position_pose.translation());
        std::cout << "[DEBUG] ICP exception (" << e.what() << "), using GPS+IMU - Translation: "
                  << new_pose.translation().transpose() << ", Quaternion: "
                  << new_pose.unit_quaternion().coeffs().transpose() << std::endl;
    }

    poses_.push_back(new_pose);
    std::cout << "[DEBUG] Pose appended to poses_, size: " << poses_.size() << std::endl;
    std::cout << "[DEBUG] Exiting RegisterFrame" << std::endl;

    return {planar_points, non_planar_points};
}

GenZICP::Vector3dVectorTuple GenZICP::RegisterFrame(const std::vector<Eigen::Vector3d> &frame) {
    std::cout << "[DEBUG] Calling RegisterFrame overload with default poses" << std::endl;
    return RegisterFrame(frame, {}, Sophus::SE3d(), Sophus::SE3d());
}

GenZICP::Vector3dVectorTuple GenZICP::Voxelize(const std::vector<Eigen::Vector3d> &frame, double adaptive_voxel_size) const {
    std::cout << "[DEBUG] Voxelizing with size: " << adaptive_voxel_size << std::endl;
    const auto frame_downsample = genz_icp::VoxelDownsample(frame, std::max(adaptive_voxel_size * 0.5, 0.02));
    const auto source = genz_icp::VoxelDownsample(frame_downsample, adaptive_voxel_size);
    return {source, frame_downsample};
}

double GenZICP::GetAdaptiveThreshold() {
    if (!HasMoved()) {
        std::cout << "[DEBUG] HasMoved false, returning initial threshold: " << config_.initial_threshold << std::endl;
        return config_.initial_threshold;
    }
    double threshold = adaptive_threshold_.ComputeThreshold();
    std::cout << "[DEBUG] Computed adaptive threshold: " << threshold << std::endl;
    return threshold;
}

Sophus::SE3d GenZICP::GetPredictionModel() const {
    Sophus::SE3d pred = Sophus::SE3d();
    const size_t N = poses_.size();
    if (N < 2) {
        std::cout << "[DEBUG] Not enough poses for prediction, returning default" << std::endl;
        return pred;
    }
    pred = poses_[N - 2].inverse() * poses_[N - 1];
    std::cout << "[DEBUG] Prediction model computed" << std::endl;
    return pred;
}

bool GenZICP::HasMoved() {
    if (poses_.empty()) {
        std::cout << "[DEBUG] Poses empty, HasMoved: false" << std::endl;
        return false;
    }
    const double motion = (poses_.front().inverse() * poses_.back()).translation().norm();
    bool moved = motion > 5.0 * config_.min_motion_th;
    std::cout << "[DEBUG] Motion: " << motion << ", HasMoved: " << (moved ? "true" : "false") << std::endl;
    return moved;
}

}  // namespace genz_icp::pipeline