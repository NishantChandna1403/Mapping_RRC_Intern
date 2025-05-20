#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cstddef>
#include <memory>
#include <regex>
#include <sophus/se3.hpp>
#include <string>
#include <vector>

// ROS 2
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/header.hpp>

namespace tf2 {

inline geometry_msgs::msg::Transform sophusToTransform(const Sophus::SE3d &T) {
    geometry_msgs::msg::Transform t;
    t.translation.x = T.translation().x();
    t.translation.y = T.translation().y();
    t.translation.z = T.translation().z();

    Eigen::Quaterniond q(T.so3().unit_quaternion());
    t.rotation.x = q.x();
    t.rotation.y = q.y();
    t.rotation.z = q.z();
    t.rotation.w = q.w();

    return t;
}

inline geometry_msgs::msg::Pose sophusToPose(const Sophus::SE3d &T) {
    geometry_msgs::msg::Pose t;
    t.position.x = T.translation().x();
    t.position.y = T.translation().y();
    t.position.z = T.translation().z();

    Eigen::Quaterniond q(T.so3().unit_quaternion());
    t.orientation.x = q.x();
    t.orientation.y = q.y();
    t.orientation.z = q.z();
    t.orientation.w = q.w();

    return t;
}

inline Sophus::SE3d transformToSophus(const geometry_msgs::msg::TransformStamped &transform) {
    const auto &t = transform.transform;
    return Sophus::SE3d(
        Sophus::SE3d::QuaternionType(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z),
        Sophus::SE3d::Point(t.translation.x, t.translation.y, t.translation.z));
}

} // namespace tf2

namespace genz_icp_ros::utils {

using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointField = sensor_msgs::msg::PointField;
using Header = std_msgs::msg::Header;

inline std::string FixFrameId(const std::string &frame_id) {
    return std::regex_replace(frame_id, std::regex("^/"), "");
}

inline int sizeOfPointField(uint8_t datatype) {
    switch (datatype) {
        case PointField::INT8:    return 1;
        case PointField::UINT8:   return 1;
        case PointField::INT16:   return 2;
        case PointField::UINT16:  return 2;
        case PointField::INT32:   return 4;
        case PointField::UINT32:  return 4;
        case PointField::FLOAT32: return 4;
        case PointField::FLOAT64: return 8;
        default:                  return 0;
    }
}

inline int addPointField(PointCloud2 &cloud_msg, const std::string &name, uint32_t count, uint8_t datatype, uint32_t offset) {
    PointField field;
    field.name = name;
    field.count = count;
    field.datatype = datatype;
    field.offset = offset;
    cloud_msg.fields.push_back(field);
    return offset + count * sizeOfPointField(datatype);
}

inline auto GetTimestampField(const PointCloud2::ConstSharedPtr msg) {
    PointField timestamp_field;
    for (const auto &field : msg->fields) {
        if (field.name == "t" || field.name == "timestamp" || field.name == "time") {
            timestamp_field = field;
            break;
        }
    }
    if (!timestamp_field.count) {
        throw std::runtime_error("Field 't', 'timestamp', or 'time' does not exist");
    }
    return timestamp_field;
}

inline auto NormalizeTimestamps(const std::vector<double> &timestamps) {
    if (timestamps.empty()) return timestamps;
    const auto [min_it, max_it] = std::minmax_element(timestamps.cbegin(), timestamps.cend());
    const double min_timestamp = *min_it;
    const double max_timestamp = *max_it;
    if (max_timestamp == min_timestamp) return std::vector<double>(timestamps.size(), 0.0);

    std::vector<double> timestamps_normalized(timestamps.size());
    std::transform(timestamps.cbegin(), timestamps.cend(), timestamps_normalized.begin(),
                   [&](const auto &timestamp) {
                       return (timestamp - min_timestamp) / (max_timestamp - min_timestamp);
                   });
    return timestamps_normalized;
}

inline auto ExtractTimestampsFromMsg(const PointCloud2::ConstSharedPtr msg,
                                    const PointField &field) {
    auto extract_timestamps =
        [&msg]<typename T>(sensor_msgs::PointCloud2ConstIterator<T> &&it) -> std::vector<double> {
        const size_t n_points = msg->height * msg->width;
        std::vector<double> timestamps;
        timestamps.reserve(n_points);
        for (size_t i = 0; i < n_points; ++i, ++it) {
            timestamps.emplace_back(static_cast<double>(*it));
        }
        return NormalizeTimestamps(timestamps);
    };

    using sensor_msgs::PointCloud2ConstIterator;
    if (field.datatype == PointField::UINT32) {
        return extract_timestamps(PointCloud2ConstIterator<uint32_t>(*msg, field.name));
    } else if (field.datatype == PointField::FLOAT32) {
        return extract_timestamps(PointCloud2ConstIterator<float>(*msg, field.name));
    } else if (field.datatype == PointField::FLOAT64) {
        return extract_timestamps(PointCloud2ConstIterator<double>(*msg, field.name));
    }

    throw std::runtime_error("Timestamp field type not supported");
}

inline PointCloud2 CreatePointCloud2Msg(const size_t n_points,
                                       const Header &header,
                                       bool timestamp = false) {
    PointCloud2 cloud_msg;
    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    cloud_msg.header = header;
    cloud_msg.header.frame_id = FixFrameId(cloud_msg.header.frame_id);
    cloud_msg.fields.clear();
    int offset = 0;
    offset = addPointField(cloud_msg, "x", 1, PointField::FLOAT32, offset);
    offset = addPointField(cloud_msg, "y", 1, PointField::FLOAT32, offset);
    offset = addPointField(cloud_msg, "z", 1, PointField::FLOAT32, offset);
    offset += sizeOfPointField(PointField::FLOAT32);
    if (timestamp) {
        offset = addPointField(cloud_msg, "time", 1, PointField::FLOAT64, offset);
        offset += sizeOfPointField(PointField::FLOAT64);
    }

    cloud_msg.point_step = offset;
    cloud_msg.row_step = cloud_msg.width * cloud_msg.point_step;
    cloud_msg.data.resize(cloud_msg.height * cloud_msg.row_step);
    modifier.resize(n_points);
    return cloud_msg;
}

inline void FillPointCloud2XYZ(const std::vector<Eigen::Vector3d> &points, PointCloud2 &msg) {
    sensor_msgs::PointCloud2Iterator<float> msg_x(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> msg_y(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> msg_z(msg, "z");
    for (size_t i = 0; i < points.size(); i++, ++msg_x, ++msg_y, ++msg_z) {
        const Eigen::Vector3d &point = points[i];
        *msg_x = point.x();
        *msg_y = point.y();
        *msg_z = point.z();
    }
}

inline void FillPointCloud2Timestamp(const std::vector<double> &timestamps, PointCloud2 &msg) {
    sensor_msgs::PointCloud2Iterator<double> msg_t(msg, "time");
    for (size_t i = 0; i < timestamps.size(); i++, ++msg_t) *msg_t = timestamps[i];
}

inline std::vector<double> GetTimestamps(const PointCloud2::ConstSharedPtr msg) {
    try {
        auto timestamp_field = GetTimestampField(msg);
        return ExtractTimestampsFromMsg(msg, timestamp_field);
    } catch (const std::runtime_error &e) {
        return {};
    }
}

inline std::vector<Eigen::Vector3d> PointCloud2ToEigen(const PointCloud2::ConstSharedPtr msg) {
    std::vector<Eigen::Vector3d> points;
    points.reserve(msg->height * msg->width);

    // Transformation matrix: Camera Optical (Z forward, X right, Y down) to ENU (X East, Y North, Z Up)
    Eigen::Matrix3d transform;
    transform << 1,  0,  0,   // Z -> X (forward to East)
                 0,  0,  1,   // X -> Y (right to North)
                 0, -1,  0;   // -Y -> Z (up to Up)

    sensor_msgs::PointCloud2ConstIterator<float> msg_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> msg_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> msg_z(*msg, "z");
    for (size_t i = 0; i < msg->height * msg->width; ++i, ++msg_x, ++msg_y, ++msg_z) {
        if (std::isfinite(*msg_x) && std::isfinite(*msg_y) && std::isfinite(*msg_z)) {
            Eigen::Vector3d point(*msg_x, *msg_y, *msg_z);
            if (point.z() <= 30.0 && point.y() <= -1.0) {
                points.emplace_back(transform * point);
            }
        }
    }
    return points;
}

inline sensor_msgs::msg::PointCloud2 EigenToPointCloud2(const std::vector<Eigen::Vector3d> &points,
                                                        const Header &header) {
    auto msg = CreatePointCloud2Msg(points.size(), header);
    FillPointCloud2XYZ(points, msg);
    return msg;
}

inline sensor_msgs::msg::PointCloud2 EigenToPointCloud2(const std::vector<Eigen::Vector3d> &points,
                                                        const Sophus::SE3d &T,
                                                        const Header &header) {
    std::vector<Eigen::Vector3d> points_t;
    points_t.resize(points.size());
    std::transform(points.cbegin(), points.cend(), points_t.begin(),
                   [&](const auto &point) { return T * point; });
    return EigenToPointCloud2(points_t, header);
}

inline sensor_msgs::msg::PointCloud2 EigenToPointCloud2(const std::vector<Eigen::Vector3d> &points,
                                                        const std::vector<double> &timestamps,
                                                        const Header &header) {
    auto msg = CreatePointCloud2Msg(points.size(), header, true);
    FillPointCloud2XYZ(points, msg);
    FillPointCloud2Timestamp(timestamps, msg);
    return msg;
}

} // namespace genz_icp_ros::utils