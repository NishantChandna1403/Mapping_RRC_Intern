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

#include <Eigen/Core>
#include <tuple>
#include <vector>

#include "genz_icp/core/Deskew.hpp"
#include "genz_icp/core/Threshold.hpp"
#include "genz_icp/core/VoxelHashMap.hpp"
#include "genz_icp/core/Registration.hpp"

namespace genz_icp::pipeline {

struct GenZConfig {
    double max_range = 30.0;
    double min_range = 0.5;
    double map_cleanup_radius = 400.0;
    int max_points_per_voxel = 1;
    double voxel_size = 0.25;
    int desired_num_voxelized_points = 2000;
    double min_motion_th = 0.1;
    double initial_threshold = 2.0;
    double planarity_threshold = 0.1;
    bool deskew = false;
    int max_num_iterations = 150;
    double convergence_criterion = 0.0001;
    double gps_accuracy = 1.0; // Default GPS accuracy (meters)
};

class GenZICP {
public:
    using Vector3dVector = std::vector<Eigen::Vector3d>;
    using Vector3dVectorTuple = std::tuple<Vector3dVector, Vector3dVector>;

public:
    explicit GenZICP(const GenZConfig &config)
        : config_(config),
          registration_(config.max_num_iterations, config.convergence_criterion),
          local_map_(config.voxel_size, config.max_range, config.map_cleanup_radius, config.planarity_threshold, config.max_points_per_voxel),
          adaptive_threshold_(config.initial_threshold, config.min_motion_th, config.max_range) {}

    GenZICP() : GenZICP(GenZConfig{}) {}

public:
    Vector3dVectorTuple RegisterFrame(const std::vector<Eigen::Vector3d> &frame);
    Vector3dVectorTuple RegisterFrame(const std::vector<Eigen::Vector3d> &frame,
                                      const std::vector<double> timestamps,
                                      const Sophus::SE3d &imu_orientation = Sophus::SE3d(),
                                      const Sophus::SE3d &local_position_pose = Sophus::SE3d());
    Vector3dVectorTuple Voxelize(const std::vector<Eigen::Vector3d> &frame, double voxel_size) const;
    double GetAdaptiveThreshold();
    Sophus::SE3d GetPredictionModel() const;
    bool HasMoved();

public:
    std::vector<Eigen::Vector3d> LocalMap() const { return local_map_.Pointcloud(); }
    std::vector<Sophus::SE3d> poses() const { return poses_; }

private:
    Sophus::SE3d fusePoses(const Sophus::SE3d& icp_pose,
                           const Sophus::SE3d& imu_orientation,
                           const Sophus::SE3d& local_position_pose,
                           size_t num_points);

    std::vector<Sophus::SE3d> poses_;
    GenZConfig config_;
    Registration registration_;
    VoxelHashMap local_map_;
    AdaptiveThreshold adaptive_threshold_;
    Sophus::SE3d initial_enu_orientation_;
    Sophus::SE3d last_imu_orientation_;
    bool first_frame_ = true;
};

}  // namespace genz_icp::pipeline