#include "GenZICP.hpp"

#include <Eigen/Core>
#include <tuple>
#include <vector>
#include <iostream>
#include <cmath>
#include <deque>
#include <algorithm>

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

    const double min_points = 50000.0;
    const double max_points = 1000000.0;
    double icp_weight = std::min(1.0, std::max(0.0, (static_cast<double>(num_points) - min_points) / (max_points - min_points)));
    double gps_imu_weight = 1.0 - icp_weight;

    std::cout << "[DEBUG] Weights - ICP: " << icp_weight << ", GPS+IMU: " << gps_imu_weight << std::endl;

    Sophus::SE3d gps_imu_pose(imu_orientation.unit_quaternion(), local_position_pose.translation());

    Eigen::Vector3d fused_trans = icp_weight * icp_pose.translation() +
                                  gps_imu_weight * gps_imu_pose.translation();

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

        new_pose = fusePoses(icp_pose, imu_orientation, local_position_pose, deskew_frame.size());

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