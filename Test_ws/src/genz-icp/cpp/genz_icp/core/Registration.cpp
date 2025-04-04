// MIT License
//
// Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill Stachniss.
// Modified by Daehan Lee, Hyungtae Lim, and Soohee Han, 2024
// Further modified to integrate Ceres Solver
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
#include "Registration.hpp"

#include <tbb/blocked_range.h>
#include <tbb/parallel_reduce.h>
#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <algorithm>
#include <cmath>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
#include <tuple>
#include <iostream>
#include <iomanip>

namespace Eigen {
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix3_6d = Eigen::Matrix<double, 3, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
}  // namespace Eigen

namespace {

inline double square(double x) { return x * x; }

// Ceres cost functions
struct PointToPlaneCostFunctor {
    PointToPlaneCostFunctor(const Eigen::Vector3d& source_point,
                           const Eigen::Vector3d& target_point,
                           const Eigen::Vector3d& normal)
        : source_point_(source_point), target_point_(target_point), normal_(normal) {}

    template <typename T>
    bool operator()(const T* const pose, T* residual) const {
        // pose is 6D vector [rx, ry, rz, tx, ty, tz]
        Eigen::Map<const Eigen::Matrix<T, 6, 1>> lie_algebra(pose);
        Sophus::SE3<T> T_perturb = Sophus::SE3<T>::exp(lie_algebra);
        
        Eigen::Matrix<T, 3, 1> transformed_point = T_perturb * source_point_.cast<T>();
        Eigen::Matrix<T, 3, 1> error = transformed_point - target_point_.cast<T>();
        residual[0] = error.dot(normal_.cast<T>());
        
        return true;
    }

private:
    const Eigen::Vector3d source_point_;
    const Eigen::Vector3d target_point_;
    const Eigen::Vector3d normal_;
};

struct PointToPointCostFunctor {
    PointToPointCostFunctor(const Eigen::Vector3d& source_point,
                           const Eigen::Vector3d& target_point)
        : source_point_(source_point), target_point_(target_point) {}

    template <typename T>
    bool operator()(const T* const pose, T* residual) const {
        // pose is 6D vector [rx, ry, rz, tx, ty, tz]
        Eigen::Map<const Eigen::Matrix<T, 6, 1>> lie_algebra(pose);
        Sophus::SE3<T> T_perturb = Sophus::SE3<T>::exp(lie_algebra);
        
        Eigen::Matrix<T, 3, 1> transformed_point = T_perturb * source_point_.cast<T>();
        Eigen::Matrix<T, 3, 1> error = transformed_point - target_point_.cast<T>();
        residual[0] = error.x();
        residual[1] = error.y();
        residual[2] = error.z();
        
        return true;
    }

private:
    const Eigen::Vector3d source_point_;
    const Eigen::Vector3d target_point_;
};

void TransformPoints(const Sophus::SE3d &T, std::vector<Eigen::Vector3d> &points) {
    std::transform(points.cbegin(), points.cend(), points.begin(),
                   [&](const auto &point) { return T * point; });
}
Sophus::SE3d SolveWithCeres(
    const std::vector<Eigen::Vector3d> &src_planar,
    const std::vector<Eigen::Vector3d> &tgt_planar,
    const std::vector<Eigen::Vector3d> &normals,
    const std::vector<Eigen::Vector3d> &src_non_planar,
    const std::vector<Eigen::Vector3d> &tgt_non_planar,
    double kernel,
    double alpha) {
    
    ceres::Problem problem;
    Eigen::Vector6d lie_algebra = Eigen::Vector6d::Zero();
    
    // Add point-to-plane residuals with alpha weighting
    for (size_t i = 0; i < src_planar.size(); ++i) {
        auto* cost_function = new ceres::AutoDiffCostFunction<PointToPlaneCostFunctor, 1, 6>(
            new PointToPlaneCostFunctor(src_planar[i], tgt_planar[i], normals[i]));
        
        // Use Cauchy loss for robustness and apply alpha weighting
        ceres::LossFunction* loss_function = new ceres::ScaledLoss(
            new ceres::CauchyLoss(kernel), 
            alpha, 
            ceres::TAKE_OWNERSHIP);
        problem.AddResidualBlock(cost_function, loss_function, lie_algebra.data());
    }
    
    // Add point-to-point residuals with (1-alpha) weighting
    for (size_t i = 0; i < src_non_planar.size(); ++i) {
        auto* cost_function = new ceres::AutoDiffCostFunction<PointToPointCostFunctor, 3, 6>(
            new PointToPointCostFunctor(src_non_planar[i], tgt_non_planar[i]));
        
        // Use Cauchy loss for robustness and apply (1-alpha) weighting
        ceres::LossFunction* loss_function = new ceres::ScaledLoss(
            new ceres::CauchyLoss(kernel), 
            1.0 - alpha, 
            ceres::TAKE_OWNERSHIP);
        problem.AddResidualBlock(cost_function, loss_function, lie_algebra.data());
    }
    
    // Configure solver
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;
    options.max_num_iterations = 10;
    options.num_threads = 4;
    
    // Solve
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    
    // Convert result to SE3
    return Sophus::SE3d::exp(lie_algebra);
}

void VisualizeStatus(size_t planar_count, size_t non_planar_count, double alpha) {
    const int bar_width = 52;
    const std::string planar_color = "\033[1;38;2;0;119;187m";
    const std::string non_planar_color = "\033[1;38;2;238;51;119m";
    const std::string alpha_color = "\033[1;32m";

    printf("\033[2J\033[1;1H"); // Clear terminal
    std::cout << "====================== GenZ-ICP (Ceres) ======================\n";
    std::cout << non_planar_color << "# of non-planar points: " << non_planar_count << ", ";
    std::cout << planar_color << "# of planar points: " << planar_count << "\033[0m\n";

    std::cout << "Unstructured  <-----  ";
    std::cout << alpha_color << "alpha: " << std::fixed << std::setprecision(3) << alpha << "\033[0m";
    std::cout << "  ----->  Structured\n";

    const int alpha_location = static_cast<int>(bar_width * alpha); 
    std::cout << "[";
    for (int i = 0; i < bar_width; ++i) {
        if (i == alpha_location) {
            std::cout << "\033[1;32m█\033[0m"; 
        } else {
            std::cout << "-"; 
        }
    }
    std::cout << "]\n";
    std::cout.flush();
}
}  // namespace

namespace genz_icp {

Registration::Registration(int max_num_iteration, double convergence_criterion)
    : max_num_iterations_(max_num_iteration), 
      convergence_criterion_(convergence_criterion) {}

std::tuple<Sophus::SE3d, std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>> Registration::RegisterFrame(
    const std::vector<Eigen::Vector3d> &frame,
    const VoxelHashMap &voxel_map,
    const Sophus::SE3d &initial_guess,
    double max_correspondence_distance,
    double kernel) {
    
    // for visualization
    std::vector<Eigen::Vector3d> final_planar_points;
    std::vector<Eigen::Vector3d> final_non_planar_points;
    final_planar_points.clear();
    final_non_planar_points.clear();

    if (voxel_map.Empty()) return std::make_tuple(initial_guess, final_planar_points, final_non_planar_points);

    std::vector<Eigen::Vector3d> source = frame;
    TransformPoints(initial_guess, source);

    // GenZ-ICP-loop
    Sophus::SE3d T_icp = Sophus::SE3d();
    for (int j = 0; j < max_num_iterations_; ++j) {
        const auto &[src_planar, tgt_planar, normals, src_non_planar, tgt_non_planar, planar_count, non_planar_count] = 
            voxel_map.GetCorrespondences(source, max_correspondence_distance);
        
        double alpha = static_cast<double>(planar_count) / static_cast<double>(planar_count + non_planar_count);
        
        // Solve with Ceres
        const Sophus::SE3d estimation = SolveWithCeres(
            src_planar, tgt_planar, normals, 
            src_non_planar, tgt_non_planar, 
            kernel, alpha);
        
        // Update points and transformation
        TransformPoints(estimation, source);
        T_icp = estimation * T_icp;
        
        // Check convergence
        Eigen::Vector6d dx = estimation.log();
        if (dx.norm() < convergence_criterion_ || j == max_num_iterations_ - 1) {
            VisualizeStatus(planar_count, non_planar_count, alpha);
            final_planar_points = src_planar;
            final_non_planar_points = src_non_planar;
            break;
        }
    }

    return std::make_tuple(T_icp * initial_guess, final_planar_points, final_non_planar_points);
}

}  // namespace genz_icp