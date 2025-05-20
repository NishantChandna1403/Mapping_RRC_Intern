// genz_icp/pipeline/GenZICP.cpp
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
    ICPResidual(const Sophus::SE3d& icp_pose) : icp_pose_(icp_pose) {}

    template <typename T>
    bool operator()(const T* const trans, const T* const quat, T* residual) const {
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> t(trans);
        Eigen::Quaternion<T> q(quat[3], quat[0], quat[1], quat[2]);

        Eigen::Vector3d icp_t = icp_pose_.translation();
        Eigen::Quaterniond icp_q = icp_pose_.unit_quaternion();

        residual[0] = t[0] - T(icp_t[0]);
        residual[1] = t[1] - T(icp_t[1]);
        residual[2] = t[2] - T(icp_t[2]);

        Eigen::Quaternion<T> q_diff = q.conjugate() * Eigen::Quaternion<T>(T(icp_q.w()), T(icp_q.x()), T(icp_q.y()), T(icp_q.z()));
        residual[3] = T(2.0) * q_diff.x();
        residual[4] = T(2.0) * q_diff.y();
        residual[5] = T(2.0) * q_diff.z();

        return true;
    }

private:
    Sophus::SE3d icp_pose_;
};

struct GPSResidual {
    GPSResidual(const Eigen::Vector3d& gps_trans) : gps_trans_(gps_trans) {}

    template <typename T>
    bool operator()(const T* const trans, T* residual) const {
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> t(trans);
        residual[0] = t[0] - T(gps_trans_[0]);
        residual[1] = t[1] - T(gps_trans_[1]);
        residual[2] = t[2] - T(gps_trans_[2]);
        return true;
    }

private:
    Eigen::Vector3d gps_trans_;
};

struct IMUPreintegrationResidual {
    IMUPreintegrationResidual(const Preintegration::PreintegratedMeasurement& meas,
                             const Sophus::SE3d& prev_pose)
        : meas_(meas), prev_pose_(prev_pose) {}

    template <typename T>
    bool operator()(const T* const trans, const T* const quat, const T* const vel, T* residual) const {
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> t(trans);
        Eigen::Quaternion<T> q(quat[3], quat[0], quat[1], quat[2]);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> v(vel);

        Eigen::Vector3d prev_t = prev_pose_.translation();
        Eigen::Quaterniond prev_q = prev_pose_.unit_quaternion();

        Eigen::Vector3d delta_p = meas_.delta_pose.translation();
        Eigen::Quaterniond delta_q = meas_.delta_pose.unit_quaternion();
        Eigen::Vector3d delta_v = meas_.delta_velocity;

        Eigen::Matrix<T, 3, 1> p_res = prev_q.cast<T>().inverse() * (t - prev_t.cast<T>()) - delta_p.cast<T>();
        residual[0] = p_res[0];
        residual[1] = p_res[1];
        residual[2] = p_res[2];

        Eigen::Quaternion<T> q_diff = (prev_q.cast<T>() * delta_q.cast<T>()).inverse() * q;
        residual[3] = T(2.0) * q_diff.x();
        residual[4] = T(2.0) * q_diff.y();
        residual[5] = T(2.0) * q_diff.z();

        Eigen::Matrix<T, 3, 1> v_res = prev_q.cast<T>().inverse() * (v - Eigen::Matrix<T, 3, 1>::Zero()) - delta_v.cast<T>();
        residual[6] = v_res[0];
        residual[7] = v_res[1];
        residual[8] = v_res[2];

        return true;
    }

private:
    Preintegration::PreintegratedMeasurement meas_;
    Sophus::SE3d prev_pose_;
};

Sophus::SE3d GenZICP::fusePosesWithCeres(const Sophus::SE3d& icp_pose,
                                         const Sophus::SE3d& attitude_orientation,
                                         const Sophus::SE3d& local_position_pose,
                                         const Vector3dVector& planar_points,
                                         const Vector3dVector& non_planar_points,
                                         double eph,
                                         double epv,
                                         const Eigen::Vector3d &velocity,
                                         const Preintegration::PreintegratedMeasurement &imu_meas) {
    std::cout << "[DEBUG] Optimizing pose with " << planar_points.size() << " planar points and "
              << non_planar_points.size() << " non-planar points" << std::endl;
    std::cout << "[DEBUG] GPS params - eph: " << eph << ", epv: " << epv
              << ", velocity: [" << velocity.transpose() << "]" << std::endl;
    std::cout << "[DEBUG] IMU - Delta time: " << imu_meas.delta_time
              << ", Delta pose: " << imu_meas.delta_pose.translation().transpose()
              << ", Delta velocity: " << imu_meas.delta_velocity.transpose() << std::endl;

    Eigen::Vector3d gps_trans = local_position_pose.translation();
    if (imu_meas.delta_time > 0.0 && velocity.norm() > 1e-6) {
        gps_trans += velocity * imu_meas.delta_time;
        std::cout << "[DEBUG] Applied velocity correction: " << (velocity * imu_meas.delta_time).transpose() << std::endl;
    }

    Eigen::Vector3d init_trans = icp_pose.translation();
    Eigen::Quaterniond init_quat = attitude_orientation.unit_quaternion();
    Eigen::Vector3d init_vel = velocity;
    double trans[3] = {init_trans[0], init_trans[1], init_trans[2]};
    double quat[4] = {init_quat.x(), init_quat.y(), init_quat.z(), init_quat.w()};
    double vel[3] = {init_vel[0], init_vel[1], init_vel[2]};

    ceres::Problem problem;
    problem.AddParameterBlock(trans, 3);
    problem.AddParameterBlock(quat, 4, new ceres::QuaternionParameterization());
    problem.AddParameterBlock(vel, 3);

    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<ICPResidual, 6, 3, 4>(new ICPResidual(icp_pose)),
        new ceres::HuberLoss(1.0), trans, quat);
    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<GPSResidual, 3, 3>(new GPSResidual(gps_trans)),
        new ceres::HuberLoss(1.0), trans);
    if (imu_meas.delta_time > 0.0 && !poses_.empty()) {
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<IMUPreintegrationResidual, 9, 3, 4, 3>(
                new IMUPreintegrationResidual(imu_meas, poses_.back())),
            nullptr, trans, quat, vel);
        std::cout << "[DEBUG] Added IMU residual block" << std::endl;
    } else {
        std::cout << "[DEBUG] Skipped IMU residual block - Delta time: " << imu_meas.delta_time
                  << ", Poses size: " << poses_.size() << std::endl;
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;
    options.max_num_iterations = config_.max_num_iterations;
    options.function_tolerance = config_.convergence_criterion;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << "[DEBUG] Ceres Solver: " << summary.BriefReport() << std::endl;

    Eigen::Vector3d fused_trans(trans[0], trans[1], trans[2]);
    Eigen::Quaterniond fused_quat(quat[3], quat[0], quat[1], quat[2]);
    fused_quat.normalize();

    std::cout << "[DEBUG] Optimized - Translation: " << fused_trans.transpose()
              << ", Quaternion: " << fused_quat.coeffs().transpose()
              << ", Velocity: " << Eigen::Vector3d(vel[0], vel[1], vel[2]).transpose() << std::endl;

    return Sophus::SE3d(fused_quat, fused_trans);
}

GenZICP::Vector3dVectorTuple GenZICP::RegisterFrame(const std::vector<Eigen::Vector3d> &frame,
                                                   const std::vector<double> timestamps,
                                                   const Sophus::SE3d &attitude_orientation,
                                                   const Sophus::SE3d &local_position_pose,
                                                   double eph,
                                                   double epv,
                                                   const Eigen::Vector3d &velocity,
                                                   const Eigen::Vector3d &delta_p,
                                                   const Eigen::Quaterniond &delta_q,
                                                   const Eigen::Vector3d &delta_v,
                                                   double sum_dt) {
    std::cout << "[DEBUG] Entering RegisterFrame (enhanced)" << std::endl;
    std::cout << "[DEBUG] Raw input frame size: " << frame.size() << std::endl;

    Preintegration::PreintegratedMeasurement imu_meas;
    imu_meas.delta_pose = Sophus::SE3d(delta_q, delta_p);
    imu_meas.delta_velocity = delta_v;
    imu_meas.delta_time = sum_dt;

    if (sum_dt > 0.0) {
        std::cout << "[DEBUG] Using IMU preintegrated data - Delta time: " << sum_dt << std::endl;
    } else {
        std::cout << "[DEBUG] No valid IMU data: Delta time: " << sum_dt << std::endl;
    }

    auto to_euler = [](const Sophus::SE3d& pose) -> Eigen::Vector3d {
        Eigen::Quaterniond q = pose.unit_quaternion();
        Eigen::Matrix3d R = q.toRotationMatrix();
        Eigen::Vector3d euler = R.eulerAngles(2, 1, 0);
        return Eigen::Vector3d(euler[2], euler[1], euler[0]);
    };

    static std::deque<Eigen::Vector3d> euler_history;
    const size_t max_history_size = 2;
    const size_t min_points_for_icp = 50;
    const double roll_threshold_deg = 2.0;
    const double pitch_threshold_deg = 2.0;
    const double yaw_threshold_deg = 2.0;

    const auto &deskew_frame = [&]() -> std::vector<Eigen::Vector3d> {
        if (!config_.deskew || timestamps.empty()) return frame;
        const size_t N = poses_.size();
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
        std::cout << "[DEBUG] No points after cropping, using GPS+Attitude" << std::endl;
        new_pose = Sophus::SE3d(attitude_orientation.unit_quaternion(), local_position_pose.translation());
        poses_.push_back(new_pose);
        Eigen::Vector3d new_euler = to_euler(new_pose);
        euler_history.push_back(new_euler);
        if (euler_history.size() > max_history_size) euler_history.pop_front();
        std::cout << "[DEBUG] GPS+Attitude - Translation: " << new_pose.translation().transpose()
                  << ", Quaternion: " << new_pose.unit_quaternion().coeffs().transpose() << std::endl;
        return {{}, {}};
    }

    if (!first_frame_ && !poses_.empty()) {
        Eigen::Vector3d current_euler = to_euler(attitude_orientation);
        Eigen::Vector3d reference_euler = to_euler(poses_.back());

        Eigen::Vector3d euler_diff;
        for (int i = 0; i < 3; ++i) {
            double diff = current_euler[i] - reference_euler[i];
            euler_diff[i] = std::atan2(std::sin(diff), std::cos(diff));
        }
        Eigen::Vector3d euler_diff_deg = euler_diff * 180.0 / M_PI;
        double roll_diff_deg = std::abs(euler_diff_deg[0]);
        double pitch_diff_deg = std::abs(euler_diff_deg[1]);
        double yaw_diff_deg = std::abs(euler_diff_deg[2]);

        std::cout << "[DEBUG] Euler differences - Roll: " << roll_diff_deg
                  << ", Pitch: " << pitch_diff_deg
                  << ", Yaw: " << yaw_diff_deg << " degrees" << std::endl;

        if (roll_diff_deg > roll_threshold_deg || pitch_diff_deg > pitch_threshold_deg || yaw_diff_deg > yaw_threshold_deg) {
            std::cout << "[DEBUG] Significant orientation change detected, using GPS+Attitude" << std::endl;
            new_pose = Sophus::SE3d(attitude_orientation.unit_quaternion(), local_position_pose.translation());
            poses_.push_back(new_pose);
            Eigen::Vector3d new_euler = to_euler(new_pose);
            euler_history.push_back(new_euler);
            if (euler_history.size() > max_history_size) euler_history.pop_front();
            std::cout << "[DEBUG] GPS+Attitude - Translation: " << new_pose.translation().transpose()
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
            voxel_size * static_cast<double>(source_tmp.size()) / static_cast<double>(config_.desired_num_voxelized_points), 0.1, 2.0);
        const auto &[source, frame_downsample] = Voxelize(cropped_frame, adaptive_voxel_size);
        voxel_size = adaptive_voxel_size;
        std::cout << "[DEBUG] Voxelized source points: " << source.size() << std::endl;

        if (source.size() < min_points_for_icp) {
            std::cout << "[DEBUG] Too few points after voxelization, using GPS+Attitude" << std::endl;
            new_pose = Sophus::SE3d(attitude_orientation.unit_quaternion(), local_position_pose.translation());
            poses_.push_back(new_pose);
            Eigen::Vector3d new_euler = to_euler(new_pose);
            euler_history.push_back(new_euler);
            if (euler_history.size() > max_history_size) euler_history.pop_front();
            std::cout << "[DEBUG] GPS+Attitude - Translation: " << new_pose.translation().transpose()
                      << ", Quaternion: " << new_pose.unit_quaternion().coeffs().transpose() << std::endl;
            return {{}, {}};
        }

        const double sigma = GetAdaptiveThreshold();

        Sophus::SE3d initial_guess;
        if (first_frame_) {
            initial_enu_attitude_ = attitude_orientation;
            initial_guess = initial_enu_attitude_;
            last_attitude_orientation_ = attitude_orientation;
            first_frame_ = false;
            std::cout << "[DEBUG] First frame initialized in ICP" << std::endl;
        } else {
            const auto prediction = GetPredictionModel();
            const auto last_pose = poses_.back();
            bool is_attitude_identity = attitude_orientation.unit_quaternion().isApprox(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0), 1e-6) &&
                                      attitude_orientation.translation().norm() < 1e-6;
            Sophus::SE3d attitude_delta = last_attitude_orientation_.inverse() * attitude_orientation;
            initial_guess = is_attitude_identity ? last_pose * prediction : last_pose * attitude_delta;
            last_attitude_orientation_ = attitude_orientation;
        }
        std::cout << "[DEBUG] Initial guess - Translation: " << initial_guess.translation().transpose()
                  << ", Quaternion: " << initial_guess.unit_quaternion().coeffs().transpose() << std::endl;

        std::cout << "[DEBUG] Running ICP registration" << std::endl;
        auto registration_result = registration_.RegisterFrame(source, local_map_, initial_guess, 3.0 * sigma, sigma / 3.0);
        std::tie(icp_pose, planar_points, non_planar_points) = registration_result;
        std::cout << "[DEBUG] ICP pose - Translation: " << icp_pose.translation().transpose()
                  << ", Quaternion: " << icp_pose.unit_quaternion().coeffs().transpose() << std::endl;

        new_pose = fusePosesWithCeres(icp_pose, attitude_orientation, local_position_pose, planar_points, non_planar_points, eph, epv, velocity, imu_meas);

        std::cout << "[DEBUG] Updating local map and threshold" << std::endl;
        const auto model_deviation = initial_guess.inverse() * new_pose;
        adaptive_threshold_.UpdateModelDeviation(model_deviation);
        local_map_.Update(frame_downsample, new_pose);
    } catch (const std::exception& e) {
        std::cout << "[DEBUG] ICP exception (" << e.what() << "), using GPS+Attitude" << std::endl;
        new_pose = Sophus::SE3d(attitude_orientation.unit_quaternion(), local_position_pose.translation());
        poses_.push_back(new_pose);
        Eigen::Vector3d new_euler = to_euler(new_pose);
        euler_history.push_back(new_euler);
        if (euler_history.size() > max_history_size) euler_history.pop_front();
        std::cout << "[DEBUG] GPS+Attitude - Translation: " << new_pose.translation().transpose()
                  << ", Quaternion: " << new_pose.unit_quaternion().coeffs().transpose() << std::endl;
        return {{}, {}};
    }

    Eigen::Vector3d new_euler = to_euler(new_pose);
    euler_history.push_back(new_euler);
    if (euler_history.size() > max_history_size) euler_history.pop_front();

    poses_.push_back(new_pose);
    std::cout << "[DEBUG] Pose appended to poses_, size: " << poses_.size() << std::endl;
    std::cout << "[DEBUG] Exiting RegisterFrame" << std::endl;

    return {planar_points, non_planar_points};
}

GenZICP::Vector3dVectorTuple GenZICP::RegisterFrame(const std::vector<Eigen::Vector3d> &frame,
                                                   const std::vector<double> timestamps,
                                                   const Sophus::SE3d &attitude_orientation,
                                                   const Sophus::SE3d &local_position_pose) {
    std::cout << "[DEBUG] Calling RegisterFrame (standard)" << std::endl;
    return RegisterFrame(frame, timestamps, attitude_orientation, local_position_pose,
                         config_.gps_accuracy, config_.gps_accuracy, Eigen::Vector3d::Zero(),
                         Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
                         Eigen::Vector3d::Zero(), 0.0);
}

GenZICP::Vector3dVectorTuple GenZICP::RegisterFrame(const std::vector<Eigen::Vector3d> &frame,
                                                   const std::vector<double> timestamps) {
    std::cout << "[DEBUG] Calling RegisterFrame (timestamps)" << std::endl;
    return RegisterFrame(frame, timestamps, Sophus::SE3d(), Sophus::SE3d());
}

GenZICP::Vector3dVectorTuple GenZICP::RegisterFrame(const std::vector<Eigen::Vector3d> &frame) {
    std::cout << "[DEBUG] Calling RegisterFrame (frame only)" << std::endl;
    return RegisterFrame(frame, {}, Sophus::SE3d(), Sophus::SE3d());
}

GenZICP::Vector3dVectorTuple GenZICP::Voxelize(const std::vector<Eigen::Vector3d> &frame, double adaptive_voxel_size) const {
    std::cout << "[DEBUG] Voxelizing with size: " << adaptive_voxel_size << std::endl;
    const auto frame_downsample = genz_icp::VoxelDownsample(frame, std::max(adaptive_voxel_size * 0.5, 0.1));
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