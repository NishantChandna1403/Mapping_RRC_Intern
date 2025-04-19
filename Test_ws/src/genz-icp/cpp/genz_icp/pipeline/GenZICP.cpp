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

#include "GenZICP.hpp"

#include <Eigen/Core>
#include <tuple>
#include <vector>
#include <iostream>
#include <cmath>
#include <deque>
#include <algorithm>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include "genz_icp/core/Deskew.hpp"
#include "genz_icp/core/Preprocessing.hpp"
#include "genz_icp/core/Registration.hpp"
#include "genz_icp/core/VoxelHashMap.hpp"

namespace genz_icp::pipeline {

struct ICPResidual {
    ICPResidual(const Sophus::SE3d& icp_pose, double weight)
        : icp_pose_(icp_pose), weight_(weight) {}

    template <typename T>
    bool operator()(const T* const trans, const T* const quat, T* residual) const {
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> t(trans);
        Eigen::Quaternion<T> q(quat[3], quat[0], quat[1], quat[2]); // w, x, y, z

        Eigen::Vector3d icp_t = icp_pose_.translation();
        Eigen::Quaterniond icp_q = icp_pose_.unit_quaternion();

        residual[0] = weight_ * (t[0] - T(icp_t[0]));
        residual[1] = weight_ * (t[1] - T(icp_t[1]));
        residual[2] = weight_ * (t[2] - T(icp_t[2]));

        Eigen::Quaternion<T> q_diff = q.conjugate() * Eigen::Quaternion<T>(T(icp_q.w()), T(icp_q.x()), T(icp_q.y()), T(icp_q.z()));
        residual[3] = weight_ * T(2.0) * q_diff.x();
        residual[4] = weight_ * T(2.0) * q_diff.y();
        residual[5] = weight_ * T(2.0) * q_diff.z();

        return true;
    }

private:
    Sophus::SE3d icp_pose_;
    double weight_;
};

struct GPSResidual {
    GPSResidual(const Eigen::Vector3d& gps_trans, double weight)
        : gps_trans_(gps_trans), weight_(weight) {}

    template <typename T>
    bool operator()(const T* const trans, T* residual) const {
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> t(trans);
        residual[0] = weight_ * (t[0] - T(gps_trans_[0]));
        residual[1] = weight_ * (t[1] - T(gps_trans_[1]));
        residual[2] = weight_ * (t[2] - T(gps_trans_[2]));
        return true;
    }

private:
    Eigen::Vector3d gps_trans_;
    double weight_;
};

struct IMUResidual {
    IMUResidual(const Eigen::Quaterniond& imu_quat, double weight)
        : imu_quat_(imu_quat), weight_(weight) {}

    template <typename T>
    bool operator()(const T* const quat, T* residual) const {
        Eigen::Quaternion<T> q(quat[3], quat[0], quat[1], quat[2]); // w, x, y, z
        Eigen::Quaternion<T> q_diff = q.conjugate() * Eigen::Quaternion<T>(T(imu_quat_.w()), T(imu_quat_.x()), T(imu_quat_.y()), T(imu_quat_.z()));
        residual[0] = weight_ * T(2.0) * q_diff.x();
        residual[1] = weight_ * T(2.0) * q_diff.y();
        residual[2] = weight_ * T(2.0) * q_diff.z();
        return true;
    }

private:
    Eigen::Quaterniond imu_quat_;
    double weight_;
};

Sophus::SE3d GenZICP::fusePosesWithCeres(const Sophus::SE3d& icp_pose,
                                         const Sophus::SE3d& imu_orientation,
                                         const Sophus::SE3d& local_position_pose,
                                         const Vector3dVector& planar_points,
                                         const Vector3dVector& non_planar_points,
                                         double eph,
                                         double epv,
                                         const Eigen::Vector3d &velocity,
                                         double dt) {
    std::cout << "[DEBUG] Optimizing pose with " << planar_points.size() << " planar points and "
              << non_planar_points.size() << " non-planar points" << std::endl;
    std::cout << "[DEBUG] GPS params - eph: " << eph << ", epv: " << epv
              << ", velocity: [" << velocity.transpose() << "], dt: " << dt << std::endl;

    // Compute weights
    const double min_points = 50000.0;
    size_t total_points = planar_points.size() + non_planar_points.size();
    double icp_uncertainty = 1.0 / (1.0 + static_cast<double>(total_points) / min_points);
    double icp_weight = 1.0 / icp_uncertainty;

    double gps_uncertainty = std::sqrt(eph * eph + epv * epv);
    if (gps_uncertainty < 0.1 || std::isnan(gps_uncertainty)) {
        gps_uncertainty = config_.gps_accuracy;
        std::cout << "[DEBUG] Invalid GPS uncertainty, using config_.gps_accuracy: " << gps_uncertainty << std::endl;
    }
    double gps_weight = 1.0 / gps_uncertainty;

    double imu_weight = 10.0;

    std::cout << "[DEBUG] Weights - ICP: " << icp_weight << ", GPS: " << gps_weight << ", IMU: " << imu_weight << std::endl;

    // Predict GPS position using velocity if dt is provided
    Eigen::Vector3d gps_trans = local_position_pose.translation();
    if (dt > 0.0 && velocity.norm() > 1e-6) {
        gps_trans += velocity * dt;
        std::cout << "[DEBUG] Applied velocity correction: " << (velocity * dt).transpose() << std::endl;
    }
    std::cout << "[DEBUG] GPS translation: " << gps_trans.transpose() << std::endl;

    // Initialize optimization state
    Eigen::Vector3d init_trans = icp_pose.translation();
    Eigen::Quaterniond init_quat = imu_orientation.unit_quaternion();
    double trans[3] = {init_trans[0], init_trans[1], init_trans[2]};
    double quat[4] = {init_quat.x(), init_quat.y(), init_quat.z(), init_quat.w()};

    // Set up Ceres problem
    ceres::Problem problem;
    problem.AddParameterBlock(trans, 3);
    problem.AddParameterBlock(quat, 4, new ceres::QuaternionParameterization());

    // Add residuals
    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<ICPResidual, 6, 3, 4>(new ICPResidual(icp_pose, icp_weight)),
        nullptr, trans, quat);
    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<GPSResidual, 3, 3>(new GPSResidual(gps_trans, gps_weight)),
        nullptr, trans);
    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<IMUResidual, 3, 4>(new IMUResidual(imu_orientation.unit_quaternion(), imu_weight)),
        nullptr, quat);

    // Solve
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;
    options.max_num_iterations = config_.max_num_iterations;
    options.function_tolerance = config_.convergence_criterion;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << "[DEBUG] Ceres Solver: " << summary.BriefReport() << std::endl;

    // Construct optimized pose
    Eigen::Vector3d fused_trans(trans[0], trans[1], trans[2]);
    Eigen::Quaterniond fused_quat(quat[3], quat[0], quat[1], quat[2]);
    fused_quat.normalize();

    std::cout << "[DEBUG] Optimized - Translation: " << fused_trans.transpose()
              << ", Quaternion: " << fused_quat.coeffs().transpose() << std::endl;

    return Sophus::SE3d(fused_quat, fused_trans);
}

GenZICP::Vector3dVectorTuple GenZICP::RegisterFrame(const std::vector<Eigen::Vector3d> &frame,
                                                   const std::vector<double> &timestamps,
                                                   const Sophus::SE3d &imu_orientation,
                                                   const Sophus::SE3d &local_position_pose,
                                                   double eph,
                                                   double epv,
                                                   const Eigen::Vector3d &velocity,
                                                   double dt) {
    std::cout << "[DEBUG] Entering RegisterFrame (enhanced)" << std::endl;
    std::cout << "[DEBUG] Raw input frame size: " << frame.size() << std::endl;

    auto to_yaw = [](const Sophus::SE3d& pose) -> double {
        Eigen::Quaterniond q = pose.unit_quaternion();
        Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);
        return euler[0];
    };

    static std::deque<double> yaw_history;
    const size_t max_history_size = 2;
    const size_t min_points_for_icp = 100;
    const double yaw_threshold_deg = 0.4;

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

    const auto &cropped_frame = Preprocess(deskew_frame, config_.max_range, config_.min_range);
    std::cout << "[DEBUG] Cropped points: " << cropped_frame.size() << std::endl;

    if (cropped_frame.empty()) {
        std::cout << "[DEBUG] No points after cropping, using GPS+IMU" << std::endl;
        new_pose = Sophus::SE3d(imu_orientation.unit_quaternion(), local_position_pose.translation());
        poses_.push_back(new_pose);
        double new_yaw = to_yaw(new_pose);
        yaw_history.push_back(new_yaw);
        if (yaw_history.size() > max_history_size) yaw_history.pop_front();
        std::cout << "[DEBUG] GPS+IMU - Translation: " << new_pose.translation().transpose()
                  << ", Quaternion: " << new_pose.unit_quaternion().coeffs().transpose() << std::endl;
        return {{}, {}};
    }

    // Check for yaw change
    if (!first_frame_ && !poses_.empty()) {
        double current_yaw = to_yaw(imu_orientation);
        double reference_yaw = to_yaw(poses_.back());
        double yaw_diff = std::atan2(std::sin(current_yaw - reference_yaw),
                                     std::cos(current_yaw - reference_yaw));
        double yaw_diff_deg = std::abs(yaw_diff * 180.0 / M_PI);

        std::cout << "[DEBUG] Yaw difference from last frame: " << yaw_diff_deg << " degrees" << std::endl;

        if (yaw_diff_deg > yaw_threshold_deg) {
            std::cout << "[DEBUG] Significant yaw change detected, skipping ICP and using GPS+IMU" << std::endl;
            new_pose = Sophus::SE3d(imu_orientation.unit_quaternion(), local_position_pose.translation());
            poses_.push_back(new_pose);
            double new_yaw = to_yaw(new_pose);
            yaw_history.push_back(new_yaw);
            if (yaw_history.size() > max_history_size) yaw_history.pop_front();
            std::cout << "[DEBUG] GPS+IMU - Translation: " << new_pose.translation().transpose()
                      << ", Quaternion: " << new_pose.unit_quaternion().coeffs().transpose() << std::endl;
            return {{}, {}};
        }
    }

    std::cout << "[DEBUG] Entering ICP + GPS + IMU mode" << std::endl;
    Sophus::SE3d icp_pose;
    try {
        static double voxel_size = config_.voxel_size;
        const auto source_tmp = genz_icp::VoxelDownsample(cropped_frame, voxel_size);
        double adaptive_voxel_size = genz_icp::Clamp(
            voxel_size * static_cast<double>(source_tmp.size()) / static_cast<double>(config_.desired_num_voxelized_points), 0.02, 2.0);
        const auto &[source, frame_downsample] = Voxelize(cropped_frame, adaptive_voxel_size);
        voxel_size = adaptive_voxel_size;
        std::cout << "[DEBUG] Voxelized source points: " << source.size() << std::endl;

        if (source.size() < min_points_for_icp) {
            std::cout << "[DEBUG] Too few points after voxelization (" << source.size()
                      << "), using GPS+IMU" << std::endl;
            new_pose = Sophus::SE3d(imu_orientation.unit_quaternion(), local_position_pose.translation());
            poses_.push_back(new_pose);
            double new_yaw = to_yaw(new_pose);
            yaw_history.push_back(new_yaw);
            if (yaw_history.size() > max_history_size) yaw_history.pop_front();
            std::cout << "[DEBUG] GPS+IMU - Translation: " << new_pose.translation().transpose()
                      << ", Quaternion: " << new_pose.unit_quaternion().coeffs().transpose() << std::endl;
            return {{}, {}};
        }

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

        new_pose = fusePosesWithCeres(icp_pose, imu_orientation, local_position_pose, planar_points, non_planar_points, eph, epv, velocity, dt);

        std::cout << "[DEBUG] Updating local map and threshold" << std::endl;
        const auto model_deviation = initial_guess.inverse() * new_pose;
        adaptive_threshold_.UpdateModelDeviation(model_deviation);
        local_map_.Update(frame_downsample, new_pose);
    } catch (const std::exception& e) {
        std::cout << "[DEBUG] ICP exception (" << e.what() << "), using GPS+IMU" << std::endl;
        new_pose = Sophus::SE3d(imu_orientation.unit_quaternion(), local_position_pose.translation());
        poses_.push_back(new_pose);
        double new_yaw = to_yaw(new_pose);
        yaw_history.push_back(new_yaw);
        if (yaw_history.size() > max_history_size) yaw_history.pop_front();
        std::cout << "[DEBUG] GPS+IMU - Translation: " << new_pose.translation().transpose()
                  << ", Quaternion: " << new_pose.unit_quaternion().coeffs().transpose() << std::endl;
        return {{}, {}};
    }

    double new_yaw = to_yaw(new_pose);
    yaw_history.push_back(new_yaw);
    if (yaw_history.size() > max_history_size) yaw_history.pop_front();

    poses_.push_back(new_pose);
    std::cout << "[DEBUG] Pose appended to poses_, size: " << poses_.size() << std::endl;
    std::cout << "[DEBUG] Exiting RegisterFrame" << std::endl;

    return {planar_points, non_planar_points};
}

GenZICP::Vector3dVectorTuple GenZICP::RegisterFrame(const std::vector<Eigen::Vector3d> &frame,
                                                   const std::vector<double> &timestamps,
                                                   const Sophus::SE3d &imu_orientation,
                                                   const Sophus::SE3d &local_position_pose) {
    std::cout << "[DEBUG] Calling RegisterFrame (standard)" << std::endl;
    return RegisterFrame(frame, timestamps, imu_orientation, local_position_pose,
                         config_.gps_accuracy, config_.gps_accuracy, Eigen::Vector3d::Zero(), 0.0);
}

GenZICP::Vector3dVectorTuple GenZICP::RegisterFrame(const std::vector<Eigen::Vector3d> &frame,
                                                   const std::vector<double> &timestamps) {
    std::cout << "[DEBUG] Calling RegisterFrame (timestamps)" << std::endl;
    return RegisterFrame(frame, timestamps, Sophus::SE3d(), Sophus::SE3d());
}

GenZICP::Vector3dVectorTuple GenZICP::RegisterFrame(const std::vector<Eigen::Vector3d> &frame) {
    std::cout << "[DEBUG] Calling RegisterFrame (frame only)" << std::endl;
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

}  