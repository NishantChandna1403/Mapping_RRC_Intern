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

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Core>
#include <tuple>
#include <vector>
#include <iostream>

#include "genz_icp/core/Deskew.hpp"
#include "genz_icp/core/Preprocessing.hpp"
#include "genz_icp/core/Registration.hpp"
#include "genz_icp/core/VoxelHashMap.hpp"

namespace genz_icp::pipeline {

// Cost function for translation (ICP + GPS)
struct TranslationCost {
    TranslationCost(const Eigen::Vector3d& measured_trans, double weight)
        : measured_trans_(measured_trans), weight_(weight) {}

    template <typename T>
    bool operator()(const T* const t, T* residual) const {
        residual[0] = T(weight_) * (t[0] - T(measured_trans_.x()));
        residual[1] = T(weight_) * (t[1] - T(measured_trans_.y()));
        residual[2] = T(weight_) * (t[2] - T(measured_trans_.z()));
        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Vector3d& measured_trans, double weight) {
        return new ceres::AutoDiffCostFunction<TranslationCost, 3, 3>(new TranslationCost(measured_trans, weight));
    }

    Eigen::Vector3d measured_trans_;
    double weight_;
};

// Cost function for orientation (IMU + ICP)
struct OrientationCost {
    OrientationCost(const Eigen::Quaterniond& measured_quat, double weight)
        : measured_quat_(measured_quat), weight_(weight) {}

    template <typename T>
    bool operator()(const T* const q, T* residual) const {
        T q_measured[4] = {T(measured_quat_.w()), T(measured_quat_.x()), T(measured_quat_.y()), T(measured_quat_.z())};
        T q_inv[4] = {q[0], -q[1], -q[2], -q[3]}; // Conjugate
        T q_diff[4];
        ceres::QuaternionProduct(q_measured, q_inv, q_diff);
        residual[0] = T(weight_) * q_diff[1];
        residual[1] = T(weight_) * q_diff[2];
        residual[2] = T(weight_) * q_diff[3];
        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Quaterniond& measured_quat, double weight) {
        return new ceres::AutoDiffCostFunction<OrientationCost, 3, 4>(new OrientationCost(measured_quat, weight));
    }

    Eigen::Quaterniond measured_quat_;
    double weight_;
};

Sophus::SE3d GenZICP::fusePosesWithCeres(const Sophus::SE3d& icp_pose,
                                         const Sophus::SE3d& imu_orientation,
                                         const Sophus::SE3d& local_position_pose,
                                         const Vector3dVector& planar_points,
                                         const Vector3dVector& non_planar_points) {
    std::cout << "[DEBUG] Fusing poses with Ceres" << std::endl;

    // Compute ICP confidence based on structured (planar) vs unstructured (non-planar) points
    double total_points = static_cast<double>(planar_points.size()) + static_cast<double>(non_planar_points.size());
    double icp_confidence = total_points > 0 ? static_cast<double>(planar_points.size()) / total_points : 0.5;
    icp_confidence = std::max(0.1, std::min(0.9, icp_confidence)); // Clamp between 0.1 and 0.9
    double gps_weight = 1.0 / (config_.gps_accuracy + 1e-6); // Inverse of GPS accuracy
    double imu_weight = 1.0; // Fixed IMU weight (assuming high reliability)

    std::cout << "[DEBUG] Weights - ICP: " << icp_confidence << ", GPS: " << gps_weight << ", IMU: " << imu_weight << std::endl;

    // Initial pose estimate (start with ICP)
    double t[3] = {icp_pose.translation().x(), icp_pose.translation().y(), icp_pose.translation().z()};
    double q[4] = {icp_pose.unit_quaternion().w(), icp_pose.unit_quaternion().x(),
                   icp_pose.unit_quaternion().y(), icp_pose.unit_quaternion().z()};

    // Set up Ceres problem
    ceres::Problem problem;
    ceres::LocalParameterization* quat_param = new ceres::QuaternionParameterization();
    problem.AddParameterBlock(q, 4, quat_param);
    problem.AddParameterBlock(t, 3);

    // Translation residuals
    ceres::CostFunction* icp_trans_cost = TranslationCost::Create(icp_pose.translation(), icp_confidence);
    problem.AddResidualBlock(icp_trans_cost, new ceres::HuberLoss(1.0), t);

    ceres::CostFunction* gps_trans_cost = TranslationCost::Create(local_position_pose.translation(), gps_weight);
    problem.AddResidualBlock(gps_trans_cost, new ceres::HuberLoss(1.0), t);

    // Orientation residuals
    ceres::CostFunction* icp_orient_cost = OrientationCost::Create(icp_pose.unit_quaternion(), icp_confidence);
    problem.AddResidualBlock(icp_orient_cost, new ceres::HuberLoss(0.1), q);

    ceres::CostFunction* imu_orient_cost = OrientationCost::Create(imu_orientation.unit_quaternion(), imu_weight);
    problem.AddResidualBlock(imu_orient_cost, new ceres::HuberLoss(0.1), q);

    // Solve
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 20;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << "[DEBUG] Ceres fusion complete: " << summary.BriefReport() << std::endl;

    // Construct fused pose
    Eigen::Quaterniond fused_quat(q[0], q[1], q[2], q[3]);
    Eigen::Vector3d fused_trans(t[0], t[1], t[2]);
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

    if (deskew_frame.size() <= 600) {
        new_pose = Sophus::SE3d(imu_orientation.unit_quaternion(), local_position_pose.translation());
        std::cout << "[DEBUG] GPS+IMU mode - Translation: " << new_pose.translation().transpose() 
                  << ", Quaternion: " << new_pose.unit_quaternion().coeffs().transpose() << std::endl;

        if (first_frame_) {
            initial_enu_orientation_ = imu_orientation;
            last_imu_orientation_ = imu_orientation;
            first_frame_ = false;
            std::cout << "[DEBUG] First frame initialized in GPS+IMU" << std::endl;
        } else {
            last_imu_orientation_ = imu_orientation;
        }

        poses_.push_back(new_pose);
        std::cout << "[DEBUG] Pose appended to poses_, size: " << poses_.size() << std::endl;
        std::cout << "[DEBUG] Exiting RegisterFrame (GPS+IMU mode)" << std::endl;
        return {planar_points, non_planar_points};
    }

    std::cout << "[DEBUG] Entering adaptive ICP + IMU + GPS mode" << std::endl;
    std::cout << "[DEBUG] Deskew frame size in adaptive mode: " << deskew_frame.size() << std::endl;
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

        // Fuse poses using structured/unstructured point confidence
        new_pose = fusePosesWithCeres(icp_pose, imu_orientation, local_position_pose, planar_points, non_planar_points);

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
    std::cout << "[DEBUG] Exiting RegisterFrame (adaptive mode)" << std::endl;

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