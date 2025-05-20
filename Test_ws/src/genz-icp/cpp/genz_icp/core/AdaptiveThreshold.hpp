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

#include <sophus/se3.hpp>

namespace genz_icp {

class AdaptiveThreshold {
public:
    AdaptiveThreshold(double initial_threshold, double min_motion_th, double max_range)
        : initial_threshold_(initial_threshold),
          min_motion_th_(min_motion_th),
          max_range_(max_range),
          threshold_(initial_threshold) {}

    double ComputeThreshold() {
        // Simple adaptive threshold computation based on motion and range
        return std::min(std::max(threshold_, min_motion_th_), max_range_);
    }

    void UpdateModelDeviation(const Sophus::SE3d& model_deviation) {
        // Update threshold based on model deviation (translation norm)
        double deviation = model_deviation.translation().norm();
        threshold_ = initial_threshold_ * (1.0 + deviation / max_range_);
    }

private:
    double initial_threshold_;
    double min_motion_th_;
    double max_range_;
    double threshold_;
};

}  // namespace genz_icp